/**

 * @file    filter_lib.c

 * @brief   Filter library implementation.

 *

 * All filters are O(1) per sample. None of them allocate memory.

 *

 * Educational note — why O(1) moving average?

 *   A naive moving average sums the whole window every sample → O(N).

 *   By keeping a running sum and subtracting the oldest sample we

 *   drop to O(1). The ring buffer gives us the oldest sample in 1

 *   load.

 */


#include "filter_lib.h"

#include <string.h>


/* -------------------------------------------------------------------- */

/*  Moving average                                                      */

/* -------------------------------------------------------------------- */


void pk_movavg_init(pk_movavg_t *f, int32_t *ring, uint16_t window)

{

    f->ring   = ring;

    f->window = window;

    f->index  = 0u;

    f->sum    = 0;

    f->count  = 0u;

    if (ring != NULL && window > 0u) {

        (void)memset(ring, 0, window * sizeof(int32_t));

    }

}


int32_t pk_movavg_push(pk_movavg_t *f, int32_t sample)

{

    if (f->ring == NULL || f->window == 0u) {

        return sample;

    }


    /* subtract the oldest, then add the new */

    f->sum -= f->ring[f->index];

    f->ring[f->index] = sample;

    f->sum += sample;

    f->index = (uint16_t)((f->index + 1u) % f->window);


    /* Until the buffer is full we divide by the number of valid samples

     * to avoid an initial low bias. */

    if (f->count < f->window) {

        f->count++;

    }

    uint16_t denom = (f->count < f->window) ? f->count : f->window;

    if (denom == 0u) denom = 1u;


    return (int32_t)(f->sum / (int32_t)denom);

}


/* -------------------------------------------------------------------- */

/*  First-order IIR                                                     */

/* -------------------------------------------------------------------- */


void pk_iir1_init(pk_iir1_t *f, float alpha)

{

    f->alpha  = alpha;

    f->prev   = 0.0f;

    f->seeded = false;

}


float pk_iir1_push(pk_iir1_t *f, float sample)

{

    if (!f->seeded) {

        f->prev   = sample;

        f->seeded = true;

        return sample;

    }

    f->prev = (f->alpha * sample) + ((1.0f - f->alpha) * f->prev);

    return f->prev;

}


/* -------------------------------------------------------------------- */

/*  Q15 EMA                                                             */

/* -------------------------------------------------------------------- */


void pk_ema_init(pk_ema_t *f, uint16_t alpha_q15)

{

    f->state     = 0;

    f->alpha_q15 = (int32_t)alpha_q15;

    f->seeded    = false;

}


int32_t pk_ema_push(pk_ema_t *f, int32_t sample)

{

    if (!f->seeded) {

        f->state  = sample << 15;   /* seed as Q15 */

        f->seeded = true;

        return sample;

    }


    /* y[n] = alpha * x[n] + (1 - alpha) * y[n-1]  in Q15.

     *

     *   state holds y[n-1] << 15

     *   alpha is in [0, 32767]

     *   so  alpha * sample       ≈ (alpha * sample)            (Q15 * Q0)

     *   and (1-alpha) * prev     ≈ ((32767-alpha) * state) >> 15

     */

    int64_t a_term = (int64_t)f->alpha_q15 * (int64_t)sample;

    int64_t b_term = ((int64_t)(32767 - f->alpha_q15) * (int64_t)f->state) >> 15;

    int64_t sum    = (a_term + b_term);


    f->state = (int32_t)sum;

    return (int32_t)(sum >> 15);

}

