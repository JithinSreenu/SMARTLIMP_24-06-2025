# Prosthetic Knee — System Architecture

> A safety-critical, FreeRTOS-based, motor + valve controller for a
> hydraulic prosthetic knee, designed around the **STM32U585AI**
> (B-U585I-IOT02A Discovery Kit).

---

## 1. Product overview

The product is a below-knee prosthetic that replaces the lost
amputated knee with an **active hydraulic damper**. Two small DC
motors drive the spool valve of the damper to:

| Phase | Required valve state | Motor A (open) | Motor B (close) |
|------|----------------------|----------------|-----------------|
| Swing / flexion | mostly open | **active** | off |
| Stance / support | mostly closed | off | **active** |
| Standing still | half-closed | ~50 % | ~50 % |

The controller reads:

* a load cell (vertical force at the socket) — ADC1 IN5 / PA0
* a potentiometer (knee flexion angle) — ADC1 IN6 / PA1
* a battery divider — ADC1 IN7 / PA2

…and drives the valve motors with PWM. It also streams telemetry
over Bluetooth (USART1) and accepts commands from a clinician GUI.

---

## 2. Functional blocks

```
                         ┌─────────────────────────┐
                         │   FreeRTOS Scheduler    │
                         │   (1 kHz SysTick)       │
                         └─────────┬───────────────┘
        ┌────────────────┬────────┴────────┬───────────────────┐
        │                │                 │                   │
        ▼                ▼                 ▼                   ▼
 ┌─────────────┐  ┌─────────────┐   ┌─────────────┐   ┌─────────────┐
 │ Safety Task │  │ Motor Task  │   │  ADC Task   │   │ Bluetooth   │
 │  (1 ms)     │  │  (1 ms)     │   │  (5 ms)     │   │ (20 ms)     │
 └──────┬──────┘  └──────┬──────┘   └──────┬──────┘   └──────┬──────┘
        │                │                 │                 │
   Watchdog +        PWM ramp         EMA filter        UART1 DMA
   heartbeat       + dead-time        + state machine    + CRC packet
        │                │                 │                 │
        ▼                ▼                 ▼                 ▼
   ┌─────────┐    ┌─────────────┐    ┌─────────────┐   ┌─────────────┐
   │  IWDG1  │    │ TIM3 CH1/CH2│    │ GPDMA1 Ch0  │   │ USART1 115k │
   └─────────┘    └─────────────┘    └─────────────┘   └─────────────┘
```

---

## 3. Wire protocol — Start | Len | Payload | Cmd | CRC | Stop

Every frame going out on USART1 (Bluetooth) **and** USART2 (DMC) is
framed like this:

```
+---------+-----+----------------+-----+-----+-----+
| 0xAA    | LEN | <LEN> bytes    | CMD | CRC |0x55 |
+---------+-----+----------------+-----+-----+-----+
   start   len    payload         cmd   crc   stop
```

* **Start = 0xAA**, **Stop = 0x55** — magic bytes for synchronisation.
* **LEN** — payload length in bytes (0..32).
* **CMD** — command ID (see `pk_cmd_t` in `packet_protocol.h`).
* **CRC** — CRC-8 over `[LEN, payload…, CMD]` with
  polynomial `0x07`, init `0x00`, no final XOR.
* Known-answer test: `CRC8("123456789") = 0xF4`.

For our **10-byte telemetry payload** the on-air frame is **15 bytes**.

### Telemetry payload layout (little-endian)

| Byte | Size | Field           | Type         | Range              |
|------|------|-----------------|--------------|--------------------|
| 0    | 1    | spool_angle     | `uint8_t`    | 0..100 %           |
| 1..2 | 2    | force           | `int16_t` LE | ±200 N             |
| 3..4 | 2    | knee_angle      | `int16_t` LE | ±200°              |
| 5..6 | 2    | moment          | `int16_t` LE | ±3000 Nm           |
| 7    | 1    | state_id        | `uint8_t`    | 0..16              |
| 8..9 | 2    | battery_mv      | `uint16_t` LE| 0..5000 mV         |

Why these widths?

* Spool 0..100 fits in 7 bits → `uint8_t` saves 1 B over `int`.
* Forces/angles/moments are signed ±→ `int16_t` covers the spec
  with comfortable headroom.
* Battery offset: raw voltage can go negative during regen, so
  we add `+5000` mV offset before transmission and the GUI undoes
  it.

The 13-byte on-air footprint means **1 full telemetry frame =
26 bits at 115 200 = 1.13 ms on the wire**, comfortably fitting
inside the 20 ms BT cycle.

---

## 4. FreeRTOS task matrix

| Task          | Period | Priority | Stack | Notes                              |
|---------------|--------|----------|-------|------------------------------------|
| vSafetyTask   | 1 ms   | HIGHEST  | 256 B | Watchdog + fault monitor           |
| vMotorTask    | 1 ms   | HIGH     | 256 B | PWM ramp, dead-time                |
| vAdcTask      | 5 ms   | ABOVE-N  | 256 B | Filter + state machine update      |
| vCommsTask    | 10 ms  | NORMAL   | 256 B | USART2 RX dispatcher               |
| vBluetoothTask| 20 ms  | NORMAL-1 | 256 B | 50 Hz telemetry push               |
| vWatchdogTask | 500 ms | LOW      | 128 B | LED heartbeat                      |

### Inter-task communication

| Producer           | Consumer           | Mechanism         |
|--------------------|--------------------|-------------------|
| vAdcTask           | vBluetoothTask     | shared `g_telemetry` (1 W / 1 R) |
| vAdcTask           | vSafetyTask        | global `g_telemetry.battery_mv` |
| ISR USART1 (idle)  | vBluetoothTask     | `xQueueSendFromISR()` |
| ISR USART2 (idle)  | vCommsTask         | `xQueueSendFromISR()` |
| ISR DMA1 (HT/TC)   | vAdcTask           | shared `pk_adc_data_t` |

---

## 5. State machine

States:

```
IDLE ──► STANDING ──► WALKING ──► STAIR_DESCENT
   ▲           │           │
   │           ▼           ▼
   └─── SITTING        STAIR_ASCENT
```

* **IDLE** — no load, valve neutral.
* **STANDING** — load > 30 N for 200 ms; valve mostly closed.
* **WALKING** — 3 heel-strikes detected; valve modulates by knee angle.
* **SITTING** — flexion > 80°, load < 10 N; valve nearly free.
* **STAIR_*** — adaptive valve profile.
* **CALIBRATION** — clinician-initiated zero + scale.
* **FAULT** — safe mode, motors off.

Every transition is logged with `(state_enter_tick)` and the previous
state is preserved in `pk_sm_ctx_t::previous`. The state ID is
broadcast in every telemetry frame, so the GUI can reconstruct the
gait phase.

---

## 6. Memory budget

| Section       | Size  | Notes                               |
|---------------|-------|-------------------------------------|
| Flash (text)  | ~46 KB| Includes all app + FreeRTOS         |
| Flash (ro)    | ~3 KB | CRC table, literals                 |
| .data         | <1 KB | Initialised globals                 |
| .bss          | ~6 KB | Buffers (DMA + RX queues + state)   |
| Heap (heap_4) | 8 KB  | Reserved in linker script           |
| Stack (MSP)   | 16 KB | Reserved in linker script           |
| Total SRAM    | ~30 KB out of 256 KB available     |

Plenty of headroom for a future TLS stack, OTA bootloader, or a
Kalman filter upgrade.

---

## 7. Safety architecture

Three independent layers:

1. **Hardware** — IWDG1 (LSI 32 kHz) resets the MCU if the safety
   task stops beating. Not connected to any external clock.
2. **Software watchdogs** — every task calls
   `pk_safety_heartbeat()` before blocking. The safety task checks
   that no task has been silent > N ticks.
3. **State machine** — every entry to a new state checks pre-conditions
   (force, angle, battery). On failure → `PK_STATE_FAULT` →
   `pk_motor_emergency_stop()`.

The safety task is the **only** code that calls `HAL_IWDG_Refresh()`.
This means a runaway task can never accidentally reset the watchdog
and hide a fault.

---

## 8. Boot sequence

```
POR ──► Reset_Handler ──► SystemInit ──► main()
                                       │
                                       ├── MPU_Config
                                       ├── HAL_Init
                                       ├── SystemClock_Config (MSI 16 MHz)
                                       ├── GPIO / DMA / ADC / UART / TIM init
                                       ├── IWDG_Init
                                       ├── pk_boot_start_tasks()
                                       │     ├── pk_adc_init
                                       │     ├── pk_motor_init
                                       │     ├── pk_uart_init
                                       │     ├── pk_packet_decoder_reset
                                       │     ├── pk_sm_init
                                       │     ├── pk_safety_init
                                       │     ├── pk_adc_calibrate
                                       │     ├── pk_adc_start
                                       │     └── xTaskCreate x6
                                       └── vTaskStartScheduler()
                                                │
                                                └── main() never returns
```

---

## 9. Build & test (offline)

```bash
# Offline host-side unit tests (no ARM toolchain needed)
cd Tests
make
./host_test

# Full firmware build (offline, requires arm-none-eabi-gcc)
cd ..
make           # build ProstheticKnee.elf
make flash     # program via openocd + ST-LINK
make size      # show section sizes
```
