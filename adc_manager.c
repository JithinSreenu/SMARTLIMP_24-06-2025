/**

 * @file    adc_manager.c

 * @brief   Internal ADC — DMA circular driver for the prosthetic knee.

 *

 * Why we do this by hand

 * ----------------------

 * The .ioc GUI is unavailable, so we manually configure the ADC

 * peripheral. Each register is documented inline.

 *

 *   ADC1 clock          : MSIS @ 16 MHz (default after reset on U585)

 *   ADC prescaler       : /1 (ADC clock = 16 MHz)

 *   Sampling time       : 79.5 cycles  →  ≈ 5 µs per channel

 *   Resolution          : 12 bits (12 effective bits at 1 MHz per spec)

 *   Oversampling        : 16x, shift 2  → 14-bit result, 1 conversion

 *                         per (16 × 5 µs) = 80 µs = 12.5 kSps per ch

 *   Scan mode           : enabled, 3 channels

 *   DMA                 : GPDMA1 channel 0, circular, half-word

 *

 * That gives us a per-channel sample rate of ~12.5 kHz which is

 * decimated down to 1 kHz inside the ADC processing task.

 */


#include "adc_manager.h"

#include "stm32u5xx_hal.h"


/* -------------------------------------------------------------------- */

/*  External HAL handles (defined in main.c)                             */

/* -------------------------------------------------------------------- */


extern ADC_HandleTypeDef  hadc1;

extern DMA_HandleTypeDef  hdma_adc1;

extern TIM_HandleTypeDef  htim2;   /* used to trigger ADC conversions */


/* -------------------------------------------------------------------- */

/*  Module-private state                                                 */

/* -------------------------------------------------------------------- */


static uint16_t        s_adc_buf[PK_ADC_BUF_LEN];

static pk_adc_data_t   s_data;

static pk_ema_t        s_ema[PK_ADC_CH__COUNT];


/* -------------------------------------------------------------------- */

/*  Channel -> rank mapping                                              */

/*  (must match the SQ1..SQ3 programmed in pk_adc_config_channels)       */

/* -------------------------------------------------------------------- */


static const uint32_t s_rank_offset[PK_ADC_CH__COUNT] = {

    0u,   /* force    -> SQ1 = buf[0]    */

    1u,   /* angle    -> SQ2 = buf[1]    */

    2u,   /* battery  -> SQ3 = buf[2]    */

};


/* -------------------------------------------------------------------- */

/*  Low-level register helpers                                           */

/* -------------------------------------------------------------------- */


static HAL_StatusTypeDef pk_adc_config_channels(void)

{

    ADC_ChannelConfTypeDef cfg = {0};


    /* Force — PA0 / IN5 */

    cfg.Channel      = ADC_CHANNEL_5;

    cfg.Rank         = ADC_REGULAR_RANK_1;

    cfg.SamplingTime = ADC_SAMPLETIME_79CYCLES_5;

    cfg.OffsetNumber = ADC_OFFSET_NONE;

    if (HAL_ADC_ConfigChannel(&hadc1, &cfg) != HAL_OK) return HAL_ERROR;


    /* Knee angle — PA1 / IN6 */

    cfg.Channel      = ADC_CHANNEL_6;

    cfg.Rank         = ADC_REGULAR_RANK_2;

    if (HAL_ADC_ConfigChannel(&hadc1, &cfg) != HAL_OK) return HAL_ERROR;


    /* Battery — PA2 / IN7 */

    cfg.Channel      = ADC_CHANNEL_7;

    cfg.Rank         = ADC_REGULAR_RANK_3;

    if (HAL_ADC_ConfigChannel(&hadc1, &cfg) != HAL_OK) return HAL_ERROR;


    return HAL_OK;

}


static HAL_StatusTypeDef pk_adc_config_oversampling(void)

{

    ADC_OversamplingTypeDef ov = {0};

    ov.Ratio         = 16u;            /* 16:1 HW averaging */

    ov.RightBitShift = ADC_RIGHTBITSHIFT_2;

    ov.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;

    ov.OversamplingStopReset = ADC_REGOVERSAMPLING_RESUMED_MODE;

    return HAL_ADCEx_ConfigOverSampling(&hadc1, &ov);

}


/* -------------------------------------------------------------------- */

/*  Public API                                                           */

/* -------------------------------------------------------------------- */


HAL_StatusTypeDef pk_adc_init(void)

{

    /* Bring up filter instances — alpha 0.20 matches the existing

     * 20 ms EMA from the legacy project (2 Hz cutoff at 50 Hz fs). */

    pk_ema_init(&s_ema[PK_ADC_CH_FORCE],   PK_ALPHA_Q15(0.20f));

    pk_ema_init(&s_ema[PK_ADC_CH_ANGLE],   PK_ALPHA_Q15(0.20f));

    pk_ema_init(&s_ema[PK_ADC_CH_BATTERY], PK_ALPHA_Q15(0.10f));


    if (pk_adc_config_channels()     != HAL_OK) return HAL_ERROR;

    if (pk_adc_config_oversampling() != HAL_OK) return HAL_ERROR;


    return HAL_OK;

}


HAL_StatusTypeDef pk_adc_start(void)

{

    /* Kick DMA in circular mode. The HAL will service the half- and

     * full-transfer interrupts into our ISR hooks. */

    if (HAL_ADC_Start_DMA(&hadc1,

                          (uint32_t *)s_adc_buf,

                          PK_ADC_BUF_LEN) != HAL_OK) {

        return HAL_ERROR;

    }

    /* Start TIM2 only if the trigger is sourced from a timer. */

    if (HAL_TIM_Base_Start(&htim2) != HAL_OK) {

        return HAL_ERROR;

    }

    return HAL_OK;

}


void pk_adc_calibrate(void)

{

    /* Calibration is mandatory on STM32U5 after every reset.

     * HAL_ADCEx_Calibration_Start handles:

     *   - Single-ended offset cal

     *   - Internal vrefint sanity check

     */

    (void)HAL_ADCEx_Calibration_Start(&hadc1);

    (void)memset(&s_data, 0, sizeof(s_data));

}


/* Called from DMA half-transfer ISR. We read the *first* half of

 * the buffer (most recent full set of samples). */

void pk_adc_dma_half_xfer_isr(void)

{

    for (uint32_t i = 0u; i < PK_ADC_CH__COUNT; i++) {

        uint16_t s = s_adc_buf[s_rank_offset[i]];

        s_data.raw[i]  = s;

        s_data.filt[i] = pk_ema_push(&s_ema[i], (int32_t)s);

    }

    s_data.ready = true;

}


/* Called from DMA full-transfer ISR. Same logic, second half. */

void pk_adc_dma_full_xfer_isr(void)

{

    const uint32_t half = PK_ADC_BUF_LEN / 2u;

    for (uint32_t i = 0u; i < PK_ADC_CH__COUNT; i++) {

        uint16_t s = s_adc_buf[half + s_rank_offset[i]];

        s_data.raw[i]  = s;

        s_data.filt[i] = pk_ema_push(&s_ema[i], (int32_t)s);

    }

    s_data.ready = true;

}


bool pk_adc_get(pk_adc_data_t *out)

{

    if (out == NULL) return false;

    if (!s_data.ready) return false;

    __BARRIER__();                  /* ensure we observe DMA-coherent data */

    *out = s_data;

    return true;

}


uint16_t pk_adc_get_raw(pk_adc_channel_t ch)

{

    if (ch >= PK_ADC_CH__COUNT) return 0u;

    return s_data.raw[ch];

}

