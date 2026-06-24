# Module-by-module developer guide

This document walks through every C file in the project, line by line,
explaining **what** the code does and **why** each decision was made.
Read this top-to-bottom on a first pass; later you will jump to the
relevant section when adding features.

---

## `Application/Inc/project_types.h`

### Fixed-width types — why?

`int` and `long` are *not* the same size across architectures. On a
Cortex-M33 with `-m32` they are 32-bit, but on a simulator they may be
64-bit. Always use `<stdint.h>` types for any field that crosses a
boundary: serialised packets, shared memory, registers.

### Range analysis

The brief lists six physical ranges. We pick the smallest unsigned
type that still covers the signed or unsigned range, plus a small
headroom factor:

| Variable     | Physical      | Code range    | Type     | Bytes |
|--------------|---------------|---------------|----------|-------|
| spool_angle  | 0..100        | 0..127        | uint8_t  | 1     |
| force        | ±200 N        | ±2 000 (0.1 N)| int16_t  | 2     |
| knee_angle   | ±200°         | ±2 000 (0.1°)| int16_t  | 2     |
| moment       | ±3 000 Nm     | ±30 000 (cNm)| int16_t  | 2     |
| state_id     | 0..16         | 0..16         | uint8_t  | 1     |
| battery_mv   | 0..5 V        | 0..6 5535 mV  | uint16_t | 2     |
| **Total payload** |        |               |          | **10** |

The 10-byte payload packs cleanly with no padding inside the
`__packed` struct. The on-air frame adds 5 framing bytes (start,
length, cmd, crc, stop) for **15 bytes per telemetry packet**.

### `STATIC_ASSERT` macro

```c
typedef char static_assertion_msg[(expr) ? 1 : -1];
```

If `expr` evaluates to false at compile time, the array dimension is
`-1`, which is illegal in C. The compiler emits an error pointing at
this line. We use it to guarantee `pk_telemetry_t` is exactly 10 bytes.

---

## `Application/Inc/packet_protocol.h` & `packet_protocol.c`

### Why Start+Stop magic bytes?

A noise spike can flip a data bit into any value — including
`0xAA`. With only a start byte, the receiver would think a new
frame just started and would buffer the noise as legitimate data.
Adding a Stop byte `0x55` creates a delimiter at *both* ends, so a
burst of noise can never construct a valid frame unless both bytes
happen to land in the right places. Probability = 2⁻¹⁶.

### Why CRC-8 with polynomial 0x07?

CRC-8 is the smallest CRC that can still detect:
* any single-bit error
* any two-bit error within an 8-bit window
* any odd number of bit errors
* all burst errors up to 8 bits

Polynomial 0x07 is the classic CRC-8 used by SMBus, PMBus, and many
automotive ECUs. The zero-init variant lets us verify the table
against the well-known answer `CRC("123456789") = 0xF4`.

### Decoder FSM

```
        ┌──────────────────────────────────┐
        │             IDLE                 │◄────────┐
        └─────────────┬────────────────────┘         │
                      │ (got 0xAA)                   │
                      ▼                              │
        ┌─────────────────────────────┐              │
        │         GOT_START           │              │
        │ (validate LEN ≤ MAX_PAYLOAD)│              │
        └─────────────┬───────────────┘              │
                      ▼                              │
        ┌─────────────────────────────┐              │
        │        GOT_PAYLOAD          │              │
        │ (count up to LEN bytes)     │              │
        └─────────────┬───────────────┘              │
                      ▼                              │
        ┌─────────────────────────────┐              │
        │          GOT_CMD            │              │
        │ (compute expected CRC)      │              │
        └─────────────┬───────────────┘              │
                      ▼                              │
        ┌─────────────────────────────┐              │
        │          GOT_CRC            │              │
        │ (compare byte to expected)  │──►RESET──────┤
        └─────────────┬───────────────┘  (mismatch)  │
                      ▼ (match)                      │
        ┌─────────────────────────────┐              │
        │          GOT_STOP           │              │
        │ (verify 0x55, deliver pdu)  │──►RESET──────┤
        └─────────────┬───────────────┘  (mismatch)  │
                      ▼ (match, delivered)           │
                  back to caller                    │
```

The decoder is byte-by-byte and re-entrant safe (uses a static
struct, no globals). It calls `pk_packet_decoder_reset()` whenever
it sees an illegal transition — this is the right behaviour for a
binary protocol because any error must drop the whole frame.

---

## `Application/Inc/filter_lib.h` & `filter_lib.c`

### Why three filters?

| Filter      | Pros                              | Cons                       | Used for      |
|-------------|-----------------------------------|----------------------------|---------------|
| Moving Avg  | Trivial to understand, no phase   | Needs full window buffer   | baseline removal |
| IIR (float) | Easy to retune at runtime         | Needs FPU (disabled)      | prototyping  |
| EMA Q15     | 1 cycle per sample, integer-only  | Coefficient quantisation   | **production**|

The motor task runs at 1 kHz without an FPU — the Q15 EMA is what
ships.

### Q15 arithmetic — how?

A Q15 number is a 16-bit signed integer representing a value in the
range [-1, 1). The real value `x` is stored as `x_q15 = x * 32768`.

For alpha = 0.20:
```
alpha_q15 = 0.20 * 32767 ≈ 6553
```

The filter becomes:
```
y[n] = (alpha * x[n] + (1-alpha) * y[n-1])
     ≈ (alpha_q15 * x[n] + (32767 - alpha_q15) * y[n-1]) >> 15
```

We do this in **64-bit** intermediate to avoid overflow during the
multiply (32 × 32 = up to 64 bits).

---

## `Application/Inc/adc_manager.h` & `adc_manager.c`

### Hardware oversampling

The U585's ADC supports up to 256× oversampling with right-shift
hardware. We use **16×** because:

* 16 samples → SNR improvement of `log2(16) ≈ 4 bits`
* Each channel's effective rate becomes 1 kHz / 16 = 62.5 Hz at our
  trigger rate — more than enough for gait
* DMA transfers one value per 16 conversions — CPU load drops to
  near zero

### Channel mapping

| ADC1 IN | Pin | Function           | Range           |
|---------|-----|--------------------|------------------|
| 5       | PA0 | Load cell force    | 0..3.3 V (0..N) |
| 6       | PA1 | Knee angle pot     | 0..3.3 V (0..°) |
| 7       | PA2 | Battery divider    | 0..3.3 V (0..5V)|

### DMA strategy

* Channel 0 of GPDMA1 in **circular** mode
* Buffer length = `CHANNELS × 2` (ping-pong halves)
* Each half-transfer triggers an ISR that copies the latest 3 samples
  into `s_data`

The ISR is **tiny** — it only fires once every 16 ADC conversions and
does three integer EMA updates. This keeps the interrupt latency
budget well below the 1 ms safety-task tick.

---

## `Application/Inc/motor_control.h` & `motor_control.c`

### Why two motors instead of one with a polarity relay?

* **Independent control** — we can hold the valve half-open with one
  motor while the other is dormant. With a single motor + relay you
  either go forward or reverse — no half-way.
* **No relay to fail** — relays can weld shut. Two brushed motors
  with a H-bridge have no mechanical latch.

### Slew-rate limiter

Each motor's duty cycle is incremented by **1 PWM tick / ms**. With
PWM period = 16 000, the worst-case ramp time from 0 to 100 % is
**16 s**. That's intentionally slow:

* avoids inrush current spikes that could brown-out the Li-ion cell
* prevents audible "clack" when the valve slams shut
* extends motor brush life

The user perceives the ramp as a smooth stiffness change, which is
exactly the prosthetic feel we want.

### Dead-time

Whenever both motors have a positive target we force the smaller one
to zero. This guarantees the H-bridge cannot drive both half-bridges
on the same side simultaneously — a classic "shoot-through" failure
that can destroy the FETs in microseconds.

---

## `Application/Inc/uart_dma.h` & `uart_dma.c`

### Idle-line DMA

The U585's USART peripheral can detect a **bus-idle** condition (no
edges for one character time) and raise an interrupt. Combined with
DMA-into-circular-buffer this is the textbook way to receive
variable-length packets without polling.

### RX path

```
ISR (idle)
   │
   ▼
for each new byte
   │
   ▼
pk_packet_decode_byte()  ─► if frame complete, push pk_uart_pkt_t
   │                          into the queue
   ▼
Re-arm DMA (HAL does this automatically)
```

The ISR is small (≈ 30 cycles per byte) and the heavy lifting
(packet decode) is done in the same ISR — that's fine because we
don't have a hard deadline shorter than the inter-byte gap.

### TX path

`pk_uart_send()` copies the frame into a per-channel scratch buffer
and starts DMA. We deliberately do **not** queue TX packets because
the BT task is the only producer — its 20 ms period gives plenty of
headroom for 1.13 ms frames at 115 200.

---

## `Application/Inc/bluetooth.h` & `bluetooth.c`

### 50 Hz telemetry cadence

The slowest gait cadence in humans is around 1 step / second, so 50
samples / second gives 50 samples per gait cycle — far above the
Nyquist requirement (≥ 2 samples per cycle) for any analysis we
plan to do downstream.

### Battery monitoring trick

The raw ADC value is 14-bit unsigned (0..16383) for a 0..5 V input.
We convert to millivolts before transmit so the GUI does not need to
know the ADC reference voltage.

### Encoder function

`encode_telemetry()` does explicit little-endian byte packing because
GCC on arm-none-eabi defaults to little-endian, but we want the
packet to be portable in case the GUI is on a big-endian host.

---

## `Application/Inc/state_machine.h` & `state_machine.c`

### Switch-case FSM vs table-driven

We chose the explicit `switch` because:

1. **MISRA-C compliance** — table-driven FSMs often rely on function
   pointers, which Rule 11.1 advises against.
2. **Auditability** — every transition is one readable `case` block.
3. **No extra RAM** — no transition table to live in `.bss`.

### Heel-strike detection

The current implementation is intentionally simple: any sample where
`force > 200` (i.e. > 20 N) is counted as a heel-strike. Three
strikes within the standing state trigger the WALKING state. A
future Kalman-filter upgrade can replace this with peak detection
inside the slope of the force signal.

### Fault injection

`pk_sm_raise_fault()` does three things:
1. Records the fault code so it is broadcast next telemetry frame.
2. Sets state to FAULT, which the safety task cannot exit.
3. Calls `pk_motor_emergency_stop()` — disconnects the H-bridge and
   zeroes both PWM channels.

---

## `Application/Inc/safety.h` & `safety.c`

### Single-writer watchdog

Every FreeRTOS-aware project needs a watchdog, but you have to
decide **who** refreshes it. We choose the safety task because:

* It already runs at the highest priority.
* It already monitors every other task's heartbeat.
* Centralising the refresh means a runaway task can't mask its own
  failure by refreshing the IWDG.

### Heartbeat algorithm

Each task stores the tick count of its last `pk_safety_heartbeat()`
call. The safety task computes:

```
silence = now - heartbeat[id]
if silence > max_silence[id]: FAULT
```

`max_silence` is set per task based on the task's nominal period
(5× the period is a common industry rule).

### IWDG vs WWDG

The IWDG (independent watchdog) clocks from the LSI 32 kHz oscillator
which is **separate** from the system clock. The WWDG is windowed and
clocks from PCLK — useless if PCLK stops. For a medical device we
**must** use IWDG.

---

## `Application/Inc/app_tasks.h` & `app_tasks.c`

### Priority ceiling

`configMAX_PRIORITIES` is set to 16. We use the top six slots for
safety-critical tasks:

```
SAFETY   = 15
MOTOR    = 14
ADC      = 13
COMMS    = 12
BT       = 11
WDT      = (tskIDLE_PRIORITY + 1) = 2
```

This leaves plenty of room for future tasks (e.g. a logging task at
priority 10).

### Shared `g_telemetry`

The ADC task writes `g_telemetry` every 5 ms; the BT task reads it
every 20 ms. We declare it `volatile` because:

* The compiler may otherwise cache the value in a register.
* The write is atomic on Cortex-M33 (single 32-bit aligned word), so
  no critical section is needed.

### Force-to-moment conversion

`moment = force * 15` is a placeholder using a fixed 1.5 m moment arm
between the load cell and the knee pivot. In production the moment
arm is a calibration parameter stored in the second flash bank.

---

## `Core/Src/main.c`

### Bypassing `HAL_InitTick`

The HAL normally grabs SysTick during `HAL_Init()` to drive
`HAL_Delay()`. FreeRTOS also wants SysTick for its 1 ms tick. If both
fight for it the system crashes inside the first millisecond.

By overriding the weak `HAL_InitTick()` with a no-op we hand SysTick
over to FreeRTOS exclusively. `HAL_Delay()` is unusable after this,
which is why **all** timing in the firmware uses `vTaskDelay()`.

### Pin map (manual, because no .ioc)

| Function         | Pin  | Alternate function |
|------------------|------|---------------------|
| LED1 (heartbeat) | PA5  | GPIO                |
| ADC1 IN5 (force) | PA0  | ADC                 |
| ADC1 IN6 (angle) | PA1  | ADC                 |
| ADC1 IN7 (batt)  | PA2  | ADC                 |
| TIM3 CH1 (mot A) | PA6  | AF2                 |
| TIM3 CH2 (mot B) | PA7  | AF2                 |
| USART1 TX        | PA9  | AF7                 |
| USART1 RX        | PA10 | AF7                 |
| USART2 TX        | PA2  | AF7 (overlaps batt) |
| USART2 RX        | PA3  | AF7                 |
| H-bridge enable  | PB0  | GPIO                |

> **Note for production hardware:** PA2 is shared between ADC1 IN7
> and USART2 TX. On the real PCB you should route USART2 to PD5/PD6
> to free PA2 for the battery divider.

### System clock

We run at **16 MHz MSI** with no PLL. This drops the core current to
~150 µA / MHz which is critical for a battery-powered prosthetic.
At 16 MHz the Cortex-M33 can still service all six tasks and a 50 Hz
DMA stream with 95 % CPU headroom.

---

## `Core/Src/startup_stm32u585.s` and `stm32u585.ld`

The vector table places FreeRTOS handlers at the standard positions
for Cortex-M33:

```
0x2C  SVCall     → vPortSVCHandler
0x38  PendSV     → xPortPendSVHandler
0x3C  SysTick    → xPortSysTickHandler
```

All other peripheral interrupts are weak-stubbed so the project
links even before the real ISRs are pulled in from the HAL.

The linker script reserves 16 KB for the main stack and 8 KB for the
FreeRTOS heap. The stack top is exported as `__StackTop` and is the
first thing loaded into MSP at reset.

---

## `Middlewares/FreeRTOS/`

The README in that directory explains which files to copy from the
official FreeRTOS-Kernel V10.x release. The only port we use is
**ARM_CM33_NTZ** (Cortex-M33, non-trustzone); the only heap is
**heap_4** (coalescing).

### Patches to apply

* `port.c::vPortSetupTimerInterrupt()` — set `configCPU_CLOCK_HZ` to
  16 000 000.
* `portasm.s` — keep `pxCurrentTCB` volatile.

---

## `Tests/`

A small but important piece of engineering. Run on any PC with plain
gcc:

```bash
cd Tests
make
./host_test
```

Expected output:

```
CRC8("123456789") = 0xF4 (expect 0xF4)
Packet round-trip OK
CRC corruption correctly rejected
Moving average OK
EMA Q15 OK
ALL TESTS PASSED
```

These five tests cover the parts of the code that are pure logic (no
HAL dependency). Anything HAL-touching must be tested on real
hardware or in QEMU/Wokwi.

---

## How to add a new feature — walkthrough

Let's say we want to add a **temperature sensor on the motor
driver**. Step-by-step:

1. **Pin allocation** — pick PB1 (no conflict). Edit `main.c::GPIO_Init`.
2. **ADC channel** — register ADC1 IN16 (PB1). Edit `adc_manager.c`
   add IN16 to the channel list and bump `PK_ADC_CH__COUNT`.
3. **Conversion** — add `raw_to_temperature()` in `app_tasks.c` and
   thread it into `g_telemetry`.
4. **Safety** — extend `pk_sm_raise_fault` to include `PK_FAULT_OVERTEMP`.
5. **Test** — add a host test for the new conversion function.
6. **Document** — update this file.
