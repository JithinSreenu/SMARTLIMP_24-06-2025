/**

 * @file    stm32u5xx_it.c

 * @brief   Interrupt service routines.

 *

 * Per project rules:

 *   - FreeRTOS hooks are declared in USER CODE BEGIN 0

 *   - Peripheral ISR overrides are placed inside the protected

 *     USER CODE BEGIN <IRQn> 0 blocks so a CubeMX regen never

 *     wipes them.

 */


#include "stm32u5xx_it.h"

#include "stm32u5xx_hal.h"

#include "FreeRTOS.h"

#include "task.h"

#include "adc_manager.h"

#include "uart_dma.h"


/* -------------------------------------------------------------------- */

/*  Externs                                                              */

/* -------------------------------------------------------------------- */


extern ADC_HandleTypeDef  hadc1;

extern DMA_HandleTypeDef  hdma_adc1;

extern UART_HandleTypeDef huart1;

extern UART_HandleTypeDef huart2;

extern DMA_HandleTypeDef  hdma_usart1_rx;

extern DMA_HandleTypeDef  hdma_usart2_rx;


/* -------------------------------------------------------------------- */

/*  Cortex-M33 fault handlers                                            */

/* -------------------------------------------------------------------- */


void NMI_Handler(void)              { }

void HardFault_Handler(void)        { while (1) {} }

void MemManage_Handler(void)        { while (1) {} }

void BusFault_Handler(void)         { while (1) {} }

void UsageFault_Handler(void)       { while (1) {} }

void DebugMon_Handler(void)         { }


/* USER CODE BEGIN 0 -------------------------------------------------------- */

/* FreeRTOS hooks — declared weak inside the kernel, redirected here. */

void xPortPendSVHandler(void)       { vPortPendSVHandler(); }

void xPortSysTickHandler(void)      { vPortSysTickHandler(); }

void vPortSVCHandler(void)          { vPortSVCHandler(); }

/* USER CODE END 0 ---------------------------------------------------------- */


/* -------------------------------------------------------------------- */

/*  SysTick — owned by FreeRTOS (HAL_InitTick returns OK, no HAL hook) */

/* -------------------------------------------------------------------- */


void SysTick_Handler(void)

{

    /* No HAL_IncTick — SysTick drives only the RTOS scheduler */

    portSYSCALL_SUSPEND_BEGIN();

    vPortSysTickHandler();

    portSYSCALL_SUSPEND_END();

}


/* USER CODE BEGIN SVC_IRQn 0 ----------------------------------------------- */

void SVC_Handler(void)              { vPortSVCHandler(); }

/* USER CODE END SVC_IRQn 0 ------------------------------------------------- */


/* USER CODE BEGIN PendSV_IRQn 0 -------------------------------------------- */

void PendSV_Handler(void)           { vPortPendSVHandler(); }

/* USER CODE END PendSV_IRQn 0 --------------------------------------------- */


/* -------------------------------------------------------------------- */

/*  DMA — ADC1 channel 0                                                */

/* -------------------------------------------------------------------- */


void DMA1_Channel0_IRQHandler(void)

{

    /* USER CODE BEGIN DMA1_Channel0_IRQn 0 -------------------------------- */

    /* Half / full transfer handling. We ignore the HAL flag directly and

     * just call our user hooks so that HAL's HAL_ADC_ConvCpltCallback

     * never fires (we don't use it). */

    uint32_t isr = DMA1->ISR;

    if (isr & (DMA_ISR_HTIF0 | DMA_ISR_TCIF0)) {

        if (isr & DMA_ISR_HTIF0) {

            pk_adc_dma_half_xfer_isr();

            DMA1->IFCR = DMA_IFCR_CHTIF0;

        }

        if (isr & DMA_ISR_TCIF0) {

            pk_adc_dma_full_xfer_isr();

            DMA1->IFCR = DMA_IFCR_CTCIF0;

        }

        /* clear the corresponding enable bits too */

        DMA1_Channel0->CCR &= ~(DMA_CCR_HTIE | DMA_CCR_TCIE);

        DMA1_Channel0->CCR |=  (DMA_CCR_HTIE | DMA_CCR_TCIE);

    }

    /* USER CODE END DMA1_Channel0_IRQn 0 ----------------------------------- */

    HAL_DMA_IRQHandler(&hdma_adc1);

}


/* -------------------------------------------------------------------- */

/*  USART1 — Bluetooth                                                  */

/* -------------------------------------------------------------------- */


void USART1_IRQHandler(void)

{

    /* USER CODE BEGIN USART1_IRQn 0 -------------------------------------- */

    uint32_t sr = USART1->ISR;

    if (sr & USART_ISR_IDLE) {

        /* clear IDLE flag by reading ISR then RDR */

        (void)USART1->ISR;

        (void)USART1->RDR;

        pk_uart_rx_isr(0u);   /* PK_UART_BT = 0 */

    }

    /* USER CODE END USART1_IRQn 0 ---------------------------------------- */

    HAL_UART_IRQHandler(&huart1);

}


/* -------------------------------------------------------------------- */

/*  USART2 — DMC                                                         */

/* -------------------------------------------------------------------- */


void USART2_IRQHandler(void)

{

    /* USER CODE BEGIN USART2_IRQn 0 -------------------------------------- */

    uint32_t sr = USART2->ISR;

    if (sr & USART_ISR_IDLE) {

        (void)USART2->ISR;

        (void)USART2->RDR;

        pk_uart_rx_isr(1u);   /* PK_UART_DMC = 1 */

    }

    /* USER CODE END USART2_IRQn 0 ---------------------------------------- */

    HAL_UART_IRQHandler(&huart2);

}


/* -------------------------------------------------------------------- */

/*  TIM2 — ADC trigger                                                 */

/* -------------------------------------------------------------------- */


void TIM2_IRQHandler(void)

{

    HAL_TIM_IRQHandler(&htim2);   /* not currently used */

}


void IWDG1_IRQHandler(void)        { /* not enabled */ }

