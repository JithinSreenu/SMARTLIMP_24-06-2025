/**
 * @file    FreeRTOSConfig.h
 * @brief   FreeRTOS configuration for STM32U585 prosthetic knee.
 *
 * Tuned for:
 *   - CPU = 16 MHz (no PLL)
 *   - 256 KB SRAM
 *   - 6 tasks (Safety / Motor / ADC / Comms / BT / WDT)
 *   - Tick = 1 ms (1 kHz scheduler)
 *
 * Heap scheme
 * -----------
 * heap_4 — coalesces adjacent free blocks on vPortFree(). Best for
 * long-running firmware where many small blocks of different sizes
 * are allocated and released.
 *
 * Important: every block of RAM that uses pvPortMalloc() must use
 * vPortFree(). We do not allow mixing with malloc/free from <stdlib.h>.
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* -------------------------------------------------------------------- */
/*  Scheduler core                                                      */
/* -------------------------------------------------------------------- */

#define configUSE_PREEMPTION                1
#define configUSE_TIME_SLICING              1
#define configUSE_PORT_OPTIMISED_TASK_SELECTION 0  /* generic, not ARMv8-M */
/* M33 supports 32 priority bits but only 8 effective for FreeRTOS.
 * configMAX_PRIORITIES must be <= 32. We use 16 to keep RAM small. */
#define configMAX_PRIORITIES                16
#define configUSE_IDLE_HOOK                 1
#define configUSE_TICK_HOOK                 0
#define configTICK_RATE_HZ                  (1000u)
#define configMINIMAL_STACK_SIZE            (128u)
#define configMAX_TASK_NAME_LEN             (8u)
#define configUSE_16_BIT_TICKS              0     /* 32-bit tick counter */
#define configIDLE_SHOULD_YIELD             1
#define configSUPPORT_DYNAMIC_ALLOCATION    1
#define configSUPPORT_STATIC_ALLOCATION     0

/* -------------------------------------------------------------------- */
/*  Co-routines (deprecated)                                            */
/* -------------------------------------------------------------------- */

#define configUSE_CO_ROUTINES               0
#define configMAX_CO_ROUTINE_PRIORITIES     1

/* -------------------------------------------------------------------- */
/*  Hooks                                                               */
/* -------------------------------------------------------------------- */

#define configUSE_MALLOC_FAILED_HOOK        1
#define configUSE_STACK_OVERFLOW_HOOK       2   /* method 2 = can continue */

/* -------------------------------------------------------------------- */
/*  Software timer                                                      */
/* -------------------------------------------------------------------- */

#define configUSE_TIMERS                    1
#define configTIMER_TASK_PRIORITY           (configMAX_PRIORITIES - 1)
#define configTIMER_QUEUE_LENGTH            4
#define configTIMER_TASK_STACK_DEPTH        (configMINIMAL_STACK_SIZE * 2)

/* -------------------------------------------------------------------- */
/*  Interrupts                                                          */
/* -------------------------------------------------------------------- */

#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY       0x0F
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY  0x05
#define configKERNEL_INTERRUPT_PRIORITY                (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << 4)
#define configMAX_SYSCALL_INTERRUPT_PRIORITY          (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << 4)

/* -------------------------------------------------------------------- */
/*  Runtime stats (off by default to save cycles)                      */
/* -------------------------------------------------------------------- */

#define configGENERATE_RUN_TIME_STATS       0
#define configUSE_TRACE_FACILITY            0
#define configUSE_STATS_FORMATTING_FUNCTIONS 0

/* -------------------------------------------------------------------- */
/*  Heap                                                                */
/* -------------------------------------------------------------------- */

#define configAPPLICATION_PROVIDES_cOutputBuffer  0

/* -------------------------------------------------------------------- */
/*  Asserts                                                             */
/* -------------------------------------------------------------------- */

#define configASSERT(x)                       \
    do {                                      \
        if ((x) == 0) {                       \
            taskDISABLE_INTERRUPTS();         \
            for (;;) {}                       \
        }                                     \
    } while (0)

/* -------------------------------------------------------------------- */
/*  Optional: CMSIS-V2 wrapper                                          */
/* -------------------------------------------------------------------- */

#define configUSE_CMSIS_V2_API               0   /* native API only */

/* -------------------------------------------------------------------- */
/*  Cortex-M33 specifics                                                */
/* -------------------------------------------------------------------- */

#define configENABLE_MPU                     0
#define configENABLE_FPU                     0
#define configENABLE_TRUSTZONE               0

#endif /* FREERTOS_CONFIG_H */
