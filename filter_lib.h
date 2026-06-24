/**
 * @file    filter_lib.h
 * @brief   Light-weight DSP filters for sensor pre-processing.
 *
 * Three filters are offered; one is selected at compile time by
 * the build configuration but all three can run concurrently if a
 * designer wants to compare them.
 *
 *   1. pk_movavg_t  — sliding-window moving average
 *   2. pk_iir1_t    — first-order IIR (a.k.a. "alpha filter" / EMA)
 *   3. pk_ema_t     — exponential moving average, same math as (2)
 *                     but using fixed-point Q15 coefficients so the
 *                     filter runs in 1 cycle on the M33.
 *
 * Why fixed-point for the EMA?
 *   - Cortex-M33 has no FPU enabled by default on the U585 class.
 *   - A floating-point multiply costs ~14 cycles; Q15 multiply +
 *     shift is 1 cycle.
 */

#ifndef FILTER_LIB_H
#define FILTER_LIB_H

#include "project_types.h"

/* -------------------------------------------------------------------- */
/*  Moving average                                                      */
/* -------------------------------------------------------------------- */

typedef struct {
    int32_t *ring;       /* caller-supplied buffer of size `window`     */
    uint16_t window;     /* number of taps                              */
    uint16_t index;      /* write cursor                                */
    int64_t  sum;        /* running sum — 64-bit to avoid wrap           */
    uint16_t count;      /* how many samples have been inserted         */
} pk_movavg_t;

void     pk_movavg_init(pk_movavg_t *f, int32_t *ring, uint16_t window);
int32_t  pk_movavg_push(pk_movavg_t *f, int32_t sample);

/* -------------------------------------------------------------------- */
/*  First-order IIR low-pass (floating-point variant)                   */
/* -------------------------------------------------------------------- */

typedef struct {
    float alpha;          /* smoothing factor 0..1                    */
    float prev;           /* previous output                          */
    bool  seeded;         /* true after first sample                  */
} pk_iir1_t;

void    pk_iir1_init(pk_iir1_t *f, float alpha);
float   pk_iir1_push(pk_iir1_t *f, float sample);

/* -------------------------------------------------------------------- */
/*  Q15 EMA — fast integer variant                                      */
/* -------------------------------------------------------------------- */

typedef struct {
    int32_t state;        /* accumulator in Q15                         */
    int32_t alpha_q15;    /* alpha in Q15 (0x0000..0x7FFF)              */
    bool    seeded;
} pk_ema_t;

void    pk_ema_init(pk_ema_t *f, uint16_t alpha_q15);
int32_t pk_ema_push(pk_ema_t *f, int32_t sample);

/* -------------------------------------------------------------------- */
/*  Helper — convert float alpha to Q15                                 */
/* -------------------------------------------------------------------- */

#define PK_ALPHA_Q15(a) ((int32_t)((a) * 32767.0f))

#endif /* FILTER_LIB_H */
