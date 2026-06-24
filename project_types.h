/**
 * @file    project_types.h
 * @brief   Prosthetic Knee Firmware — Common Types & Macros
 * @author  Mavis (embedded architect)
 * @version 1.0.0
 * @target  STM32U585AI (B-U585I-IOT02A)
 *
 * Purpose
 * -------
 * Centralised, fixed-width types and project-wide macros.
 * Using stdint types instead of native C types guarantees the same
 * bit width on every compiler / optimisation level (MISRA-C 2012 Rule 4.6).
 *
 * Optimisation notes
 * ------------------
 * All telemetry variables are packed into the smallest unsigned type
 * that still covers the requested physical range. This is critical on
 * Cortex-M33 because:
 *   - SRAM is scarce (the U585 has only ~256 KB total SRAM)
 *   - Each byte saved in a struct is one less byte to DMA / transmit
 *   - Smaller types align to smaller aligned offsets inside packets
 *
 * Range analysis (with chosen unsigned type):
 *
 *   Variable            Physical Range   Bits Needed   Chosen Type
 *   -----------------   ---------------  ------------  ----------------
 *   Spool Angle         0 .. 100         7 bits        uint8_t   (0..255)
 *   Force Reading       -200 .. +200     9 bits sign   int16_t   (-32768..+32767)
 *   Knee Angle          -200 .. +200     9 bits sign   int16_t
 *   Moment Reading      -3000 .. +3000   13 bits sign  int16_t   (still fits)
 *   State ID            0 .. 16          5 bits        uint8_t
 *   Battery Monitor     -5 V .. +5 V     ~10 bits      int16_t   (mV resolution)
 *
 * Casting rule: signed ranges are transmitted as **two's complement
 * little-endian** raw bytes. The receiver decodes with the same cast.
 */

#ifndef PROJECT_TYPES_H
#define PROJECT_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* -------------------------------------------------------------------- */
/*  Compiler / MISRA helpers                                            */
/* -------------------------------------------------------------------- */

#ifndef __weak
#define __weak   __attribute__((weak))
#endif

#ifndef __packed
#define __packed __attribute__((packed))
#endif

/* Inline assembly barrier — used to defeat the compiler reordering
 * a memory write past a hardware register access. */
#ifndef __BARRIER__
#define __BARRIER__() __asm__ volatile("" ::: "memory")
#endif

/* Compile-time assertion — if the expression is false the build
 * breaks with "error: size of array is negative". Used to validate
 * struct sizes at compile time. */
#define STATIC_ASSERT(expr, msg) \
    typedef char static_assertion_##msg[(expr) ? 1 : -1]

/* -------------------------------------------------------------------- */
/*  Project telemetry value types                                       */
/* -------------------------------------------------------------------- */

/**
 * @brief Spool valve opening percentage (0..100 %).
 *        Unsigned 8-bit is more than enough.
 */
typedef uint8_t  pk_spool_angle_t;

/**
 * @brief Force reading in raw units (signed, scale = 0.1 N).
 *        Range -2000..+2000 fits comfortably in int16_t.
 */
typedef int16_t  pk_force_t;

/**
 * @brief Knee flexion angle in 0.1° units. Range -2000..+2000
 *        covers -200°..+200° which is the mechanical limit of the
 *        prosthetic knee (real knees only flex to ~135°).
 */
typedef int16_t  pk_knee_angle_t;

/**
 * @brief Joint moment in 0.01 Nm units. Range -30000..+30000
 *        represents -300 Nm..+300 Nm which covers human gait peaks.
 */
typedef int16_t  pk_moment_t;

/**
 * @brief State machine identifier. 0..16 fits in 5 bits, we use
 *        uint8_t for natural alignment.
 */
typedef uint8_t  pk_state_id_t;

/**
 * @brief Battery voltage in millivolts. Range 0..+65535 mV
 *        covers any realistic Li-ion pack (single cell to 14S).
 *        For the signed -5..+5 V spec we just use unsigned and
 *        add 30000 mV offset before transmitting.
 */
typedef uint16_t pk_battery_mv_t;

/* -------------------------------------------------------------------- */
/*  Compound telemetry struct                                           */
/* -------------------------------------------------------------------- */
/**
 * @brief Single telemetry frame.
 *
 * Total payload size on the wire (after packing):
 *   spool (1) + force (2) + knee (2) + moment (2) +
 *   state (1) + battery (2) + crc (1) + start (1) + stop (1) = 13 bytes.
 *
 * We use __packed so the compiler does not insert padding.
 * This keeps the on-air length deterministic and the DMA setup
 * trivial.
 */
typedef struct __packed {
    pk_spool_angle_t spool_angle;   /* byte  0       */
    pk_force_t       force;         /* bytes 1..2 LE */
    pk_knee_angle_t  knee_angle;    /* bytes 3..4 LE */
    pk_moment_t      moment;        /* bytes 5..6 LE */
    pk_state_id_t    state_id;      /* byte  7       */
    pk_battery_mv_t  battery_mv;    /* bytes 8..9 LE */
} pk_telemetry_t;

STATIC_ASSERT(sizeof(pk_telemetry_t) == 10, telemetry_size_must_be_10_bytes);

/* -------------------------------------------------------------------- */
/*  State machine IDs                                                   */
/* -------------------------------------------------------------------- */

typedef enum {
    PK_STATE_IDLE            = 0,
    PK_STATE_STANDING        = 1,
    PK_STATE_WALKING         = 2,
    PK_STATE_SITTING         = 3,
    PK_STATE_STAIR_ASCENT    = 4,
    PK_STATE_STAIR_DESCENT   = 5,
    PK_STATE_CALIBRATION     = 6,
    PK_STATE_FAULT           = 7,
    PK_STATE_SLEEP           = 8,
    PK_STATE_BOOT            = 9,
    PK_STATE_MAX             = 16   /* hard upper bound used in code */
} pk_state_t;

/* -------------------------------------------------------------------- */
/*  Error / fault codes                                                 */
/* -------------------------------------------------------------------- */

typedef enum {
    PK_FAULT_NONE              = 0x00,
    PK_FAULT_ADC_TIMEOUT       = 0x01,
    PK_FAULT_ADC_OVERRANGE     = 0x02,
    PK_FAULT_SENSOR_DISCONNECT = 0x03,
    PK_FAULT_UART_TIMEOUT      = 0x04,
    PK_FAULT_MOTOR_STALL       = 0x05,
    PK_FAULT_BATTERY_LOW       = 0x06,
    PK_FAULT_BATTERY_CRITICAL  = 0x07,
    PK_FAULT_WATCHDOG          = 0x08,
    PK_FAULT_STACK_OVERFLOW    = 0x09,
    PK_FAULT_HEAP_FULL         = 0x0A,
    PK_FAULT_CRC_MISMATCH      = 0x0B,
    PK_FAULT_OVERTEMP          = 0x0C
} pk_fault_t;

#endif /* PROJECT_TYPES_H */
