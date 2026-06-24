/**
 * @file    adc_manager.h
 * @brief   Internal ADC manager — DMA circular, oversampling, calibration.
 *
 * Pin map (B-U585I-IOT02A, all hand-routed because we cannot use .ioc)
 *
 *   PA0  -> ADC1 IN5   — load cell force (already signal conditioned)
 *   PA1  -> ADC1 IN6   — knee angle potentiometer (0..3.3 V)
 *   PA2  -> ADC1 IN7   — battery voltage divider (×0.2 ratio)
 *
 * Sampling rate: 1 kHz per channel via TIM2 trigger @ 3 kHz total
 * Conversion: 12-bit, hardware oversampling 16× → 16-bit effective
 * DMA: GPDMA1 channel 0, circular, half-word transfers
 *
 * Educational note — why a hardware oversampler?
 *   1. Averaging N samples in HW reduces noise by sqrt(N).
 *      16 samples → ~12 dB SNR improvement.
 *   2. The CPU is freed for the gait algorithm.
 *   3. DMA only fires once every 16 conversions.
 */

#ifndef ADC_MANAGER_H
#define ADC_MANAGER_H

#include "project_types.h"
#include <stdint.h>

#define PK_ADC_CHANNELS        (3u)          /* force / angle / battery */
#define PK_ADC_SAMPLES_PER_CH  (16u)         /* HW oversampling        */
#define PK_ADC_BUF_LEN         (PK_ADC_CHANNELS * PK_ADC_SAMPLES_PER_CH * 2u) /* ping-pong */

typedef enum {
    PK_ADC_CH_FORCE    = 0,
    PK_ADC_CH_ANGLE    = 1,
    PK_ADC_CH_BATTERY  = 2,
    PK_ADC_CH__COUNT   = 3
} pk_adc_channel_t;

typedef struct {
    uint16_t raw[PK_ADC_CH__COUNT];      /* latest raw value          */
    int32_t  filt[PK_ADC_CH__COUNT];     /* filtered value (IIR EMA)  */
    bool     ready;                      /* at least one full DMA HT  */
    uint32_t overruns;                   /* debug counter             */
} pk_adc_data_t;

/* Public API */
HAL_StatusTypeDef pk_adc_init(void);
HAL_StatusTypeDef pk_adc_start(void);
void              pk_adc_calibrate(void);
bool              pk_adc_get(pk_adc_data_t *out);
uint16_t          pk_adc_get_raw(pk_adc_channel_t ch);

/* ISR-side hooks — called from the half-transfer / full-transfer
 * DMA interrupts (defined in stm32u5xx_it.c) */
void pk_adc_dma_half_xfer_isr(void);
void pk_adc_dma_full_xfer_isr(void);

#endif /* ADC_MANAGER_H */
