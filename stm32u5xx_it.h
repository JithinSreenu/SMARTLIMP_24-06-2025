/**
 * @file    stm32u5xx_it.h
 * @brief   Interrupt service routine prototypes.
 *
 * Per project rules this header holds *only* prototypes, never
 * executable code, so it cannot trigger "multi-definition" linker
 * errors during a CubeMX regeneration.
 */

#ifndef STM32U5XX_IT_H
#define STM32U5XX_IT_H

#ifdef __cplusplus
extern "C" {
#endif

void NMI_Handler                    (void);
void HardFault_Handler              (void);
void MemManage_Handler              (void);
void BusFault_Handler               (void);
void UsageFault_Handler             (void);
void DebugMon_Handler               (void);
void SVC_Handler                    (void);
void PendSV_Handler                 (void);
void SysTick_Handler                (void);

/* Peripherals used by the firmware */
void DMA1_Channel0_IRQHandler       (void);   /* ADC1 DMA */
void USART1_IRQHandler              (void);   /* Bluetooth */
void USART2_IRQHandler              (void);   /* DMC */
void TIM2_IRQHandler                (void);
void IWDG1_IRQHandler               (void);

#ifdef __cplusplus
}
#endif

#endif /* STM32U5XX_IT_H */
