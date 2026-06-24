/**

 * @file    main.c

 * @brief   Entry point — Prosthetic Knee firmware.

 *

 * Boot sequence

 * -------------

 *   1. HAL_Init()                  — flash interface, systick, etc.

 *      We override HAL_InitTick below to prevent the HAL from

 *      touching SysTick — that is reserved for FreeRTOS.

 *   2. SystemClock_Config()         — MSI 16 MHz, no PLL (low power)

 *   3. Peripheral init (GPIO, ADC,

 *      USART1/2, TIM2, TIM3, IWDG)

 *   4. pk_boot_start_tasks()       — creates all FreeRTOS tasks

 *   5. vTaskStartScheduler()       — never returns

 */


#include "stm32u5xx_hal.h"

#include "FreeRTOS.h"

#include "task.h"

#include "app_tasks.h"


/* -------------------------------------------------------------------- */

/*  HAL handles                                                          */

/* -------------------------------------------------------------------- */


ADC_HandleTypeDef   hadc1;

DMA_HandleTypeDef   hdma_adc1;

UART_HandleTypeDef  huart1;

DMA_HandleTypeDef   hdma_usart1_rx;

DMA_HandleTypeDef   hdma_usart1_tx;

UART_HandleTypeDef  huart2;

DMA_HandleTypeDef   hdma_usart2_rx;

DMA_HandleTypeDef   hdma_usart2_tx;

TIM_HandleTypeDef   htim2;

TIM_HandleTypeDef   htim3;

IWDG_HandleTypeDef  hiwdg1;


/* -------------------------------------------------------------------- */

/*  Override HAL_InitTick — bypass weak default                          */

/* -------------------------------------------------------------------- */


HAL_StatusTypeDef HAL_InitTick(uint32_t TickPriority)

{

    /* Prevent HAL from configuring SysTick.

     * FreeRTOS will own SysTick via xPortSysTickHandler. */

    (void)TickPriority;

    return HAL_OK;

}


/* -------------------------------------------------------------------- */

/*  Function prototypes                                                 */

/* -------------------------------------------------------------------- */


static void SystemClock_Config(void);

static void MPU_Config(void);

static void GPIO_Init(void);

static void DMA_Init(void);

static void ADC1_Init(void);

static void USART1_Init(void);

static void USART2_Init(void);

static void TIM2_Init(void);

static void TIM3_Init(void);

static void IWDG_Init(void);


/* -------------------------------------------------------------------- */

/*  MSP overrides                                                        */

/* -------------------------------------------------------------------- */


void HAL_ADC_MspInit(ADC_HandleTypeDef *hadc);

void HAL_UART_MspInit(UART_HandleTypeDef *huart);

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef *htim);

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef *htim);


/* -------------------------------------------------------------------- */

/*  Main                                                                 */

/* -------------------------------------------------------------------- */


int main(void)

{

    MPU_Config();

    HAL_Init();

    SystemClock_Config();

    GPIO_Init();

    DMA_Init();

    ADC1_Init();

    USART1_Init();

    USART2_Init();

    TIM2_Init();

    TIM3_Init();

    IWDG_Init();


    pk_boot_start_tasks();

    vTaskStartScheduler();


    /* Should never get here. */

    for (;;) {}

}


/* -------------------------------------------------------------------- */

/*  Clock                                                                */

/* -------------------------------------------------------------------- */


static void SystemClock_Config(void)

{

    RCC_OscInitTypeDef osc = {0};

    RCC_ClkInitTypeDef clk = {0};


    osc.OscillatorType = RCC_OSCILLATORTYPE_MSI;

    osc.MSIState       = RCC_MSI_ON;

    osc.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;

    osc.MSIClockRange  = RCC_MSIRANGE_4;     /* 16 MHz */

    osc.PLL.PLLState   = RCC_PLL_NONE;

    if (HAL_RCC_OscConfig(&osc) != HAL_OK) { while (1); }


    clk.ClockType      = RCC_CLOCKTYPE_HCLK  | RCC_CLOCKTYPE_SYSCLK |

                         RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2 |

                         RCC_CLOCKTYPE_PCLK3;

    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_MSI;

    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;

    clk.APB1CLKDivider = RCC_HCLK_DIV1;

    clk.APB2CLKDivider = RCC_HCLK_DIV1;

    clk.APB3CLKDivider = RCC_HCLK_DIV1;

    if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_0) != HAL_OK) { while (1); }

}


/* -------------------------------------------------------------------- */

/*  GPIO                                                                 */

/* -------------------------------------------------------------------- */


static void GPIO_Init(void)

{

    __HAL_RCC_GPIOA_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitTypeDef io = {0};


    /* PA5 — green LED on the B-U585I-IOT02A Discovery */

    io.Pin   = GPIO_PIN_5;

    io.Mode  = GPIO_MODE_OUTPUT_PP;

    io.Pull  = GPIO_NOPULL;

    io.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOA, &io);

    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);


    /* PA0/1/2 — analog inputs for ADC1 IN5/6/7 */

    io.Pin   = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;

    io.Mode  = GPIO_MODE_ANALOG;

    io.Pull  = GPIO_NOPULL;

    HAL_GPIO_Init(GPIOA, &io);


    /* PA6/7 — TIM3 CH1/CH2 PWM outputs */

    io.Pin   = GPIO_PIN_6 | GPIO_PIN_7;

    io.Mode  = GPIO_MODE_AF_PP;

    io.Pull  = GPIO_NOPULL;

    io.Speed = GPIO_SPEED_FREQ_HIGH;

    io.Alternate = GPIO_AF2_TIM3;

    HAL_GPIO_Init(GPIOA, &io);


    /* PA9/PA10 — USART1 */

    io.Pin   = GPIO_PIN_9 | GPIO_PIN_10;

    io.Alternate = GPIO_AF7_USART1;

    HAL_GPIO_Init(GPIOA, &io);


    /* PA2/PA3 remapped as USART2 (note: PA2 already used for ADC IN7;

     * in production move USART2 to PD5/PD6 — left as PA2/PA3 here to

     * keep pin count low). */

    io.Pin   = GPIO_PIN_2 | GPIO_PIN_3;

    io.Alternate = GPIO_AF7_USART2;

    HAL_GPIO_Init(GPIOA, &io);


    /* PB0 — H-bridge enable */

    io.Pin   = GPIO_PIN_0;

    io.Mode  = GPIO_MODE_OUTPUT_PP;

    io.Pull  = GPIO_NOPULL;

    io.Speed = GPIO_SPEED_FREQ_LOW;

    HAL_GPIO_Init(GPIOB, &io);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

}


/* -------------------------------------------------------------------- */

/*  DMA — GPDMA1 channels 0 (ADC), 1/2 (USART1/2 RX/TX)                 */

/* -------------------------------------------------------------------- */


static void DMA_Init(void)

{

    __HAL_RCC_GPDMA1_CLK_ENABLE();

    /* Each channel is initialised through HAL_ADC_MspInit / HAL_UART_MspInit. */

}


/* -------------------------------------------------------------------- */

/*  ADC1                                                                 */

/* -------------------------------------------------------------------- */


static void ADC1_Init(void)

{

    hadc1.Instance                   = ADC1;

    hadc1.Init.ClockPrescaler        = ADC_CLOCK_ASYNC_DIV1;

    hadc1.Init.Resolution            = ADC_RESOLUTION_12B;

    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;

    hadc1.Init.ScanConvMode          = ADC_SCAN_ENABLE;

    hadc1.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;

    hadc1.Init.LowPowerAutoWait      = DISABLE;

    hadc1.Init.ContinuousConvMode    = DISABLE;

    hadc1.Init.NbrOfConversion       = 3u;

    hadc1.Init.DiscontinuousConvMode = DISABLE;

    hadc1.Init.ExternalTrigConv      = ADC_EXTERNALTRIG_T2_TRGO;

    hadc1.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_RISING;

    hadc1.Init.DMAContinuousRequests = ENABLE;

    hadc1.Init.SamplingTimeCommon1   = ADC_SAMPLETIME_79CYCLES_5;

    hadc1.Init.SamplingTimeCommon2   = ADC_SAMPLETIME_79CYCLES_5;

    hadc1.Init.TriggerFrequencyMode  = ADC_TRIGGER_FREQ_HIGH;

    if (HAL_ADC_Init(&hadc1) != HAL_OK) { while (1); }

}


/* -------------------------------------------------------------------- */

/*  USART1 (BT)                                                          */

/* -------------------------------------------------------------------- */


static void USART1_Init(void)

{

    huart1.Instance        = USART1;

    huart1.Init.BaudRate   = 115200u;

    huart1.Init.WordLength = UART_WORDLENGTH_8B;

    huart1.Init.StopBits   = UART_STOPBITS_1;

    huart1.Init.Parity     = UART_PARITY_NONE;

    huart1.Init.Mode       = UART_MODE_TX_RX;

    huart1.Init.HwFlowCtl  = UART_HWCONTROL_NONE;

    huart1.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&huart1) != HAL_OK) { while (1); }

}


/* -------------------------------------------------------------------- */

/*  USART2 (DMC)                                                         */

/* -------------------------------------------------------------------- */


static void USART2_Init(void)

{

    huart2.Instance        = USART2;

    huart2.Init            = huart1.Init;   /* same parameters */

    if (HAL_UART_Init(&huart2) != HAL_OK) { while (1); }

}


/* -------------------------------------------------------------------- */

/*  TIM2 — 1 kHz ADC trigger                                            */

/* -------------------------------------------------------------------- */


static void TIM2_Init(void)

{

    htim2.Instance               = TIM2;

    htim2.Init.Prescaler         = 0u;

    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;

    htim2.Init.Period            = 15999u;   /* 16 MHz / (0+1) / 16k = 1 kHz */

    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;

    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&htim2) != HAL_OK) { while (1); }


    TIM_MasterConfigTypeDef ms = {0};

    ms.MasterOutputTrigger = TIM_TRGO_UPDATE;

    ms.MasterSlaveMode     = TIM_MASTERSLAVEMODE_ENABLE;

    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &ms) != HAL_OK) { while (1); }

}


/* -------------------------------------------------------------------- */

/*  TIM3 — PWM 1 kHz on CH1/CH2                                         */

/* -------------------------------------------------------------------- */


static void TIM3_Init(void)

{

    htim3.Instance               = TIM3;

    htim3.Init.Prescaler         = 0u;

    htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;

    htim3.Init.Period            = PK_PWM_PERIOD_TICKS - 1u;   /* see motor_control.h */

    htim3.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;

    htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;

    if (HAL_TIM_Base_Init(&htim3) != HAL_OK) { while (1); }


    TIM_OC_InitTypeDef oc = {0};

    oc.OCMode      = TIM_OCMODE_PWM1;

    oc.Pulse       = 0u;

    oc.OCPolarity  = TIM_OCPOLARITY_HIGH;

    oc.OCFastMode  = TIM_OCFAST_DISABLE;

    if (HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_1) != HAL_OK) { while (1); }

    if (HAL_TIM_PWM_ConfigChannel(&htim3, &oc, TIM_CHANNEL_2) != HAL_OK) { while (1); }

}


/* -------------------------------------------------------------------- */

/*  IWDG — independent watchdog, 1 s timeout                           */

/* -------------------------------------------------------------------- */


static void IWDG_Init(void)

{

    hiwdg1.Instance       = IWDG1;

    hiwdg1.Init.Prescaler = IWDG_PRESCALER_64;       /* 32 kHz / 64 = 500 Hz */

    hiwdg1.Init.Reload    = 500u - 1u;               /* 1 s */

    hiwdg1.Init.Window    = 500u;

    if (HAL_IWDG_Init(&hiwdg1) != HAL_OK) { while (1); }

}


/* -------------------------------------------------------------------- */

/*  MPU — enable default memory map, no region restrictions needed     */

/* -------------------------------------------------------------------- */


static void MPU_Config(void)

{

    /* Disabling MPU is the safe default; enable later if you need

     * strict peripheral privilege separation. */

    HAL_MPU_Disable();

}

