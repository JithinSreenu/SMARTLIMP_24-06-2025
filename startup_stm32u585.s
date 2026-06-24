/**
 * @file    startup_stm32u585.s
 * @brief   Cortex-M33 vector table + reset for STM32U585AI.
 *
 * Hand-written because we cannot pull the CMSIS device pack.
 * Memory map (taken from RM0456 rev 1, Table 4):
 *
 *   Flash  : 0x08000000  ..  0x080FFFFF  (1 MB)
 *   SRAM   : 0x20000000  ..  0x2003FFFF  (256 KB)
 *
 * We use only the first 256 KB of flash — plenty for a single
 * firmware image. The unused upper area is reserved for future
 * OTA / Bank 2 telemetry logs.
 */

  .syntax unified
  .cpu cortex-m33
  .fpu fpv5-sp-d16
  .thumb

/* -------------------------------------------------------------------- */
/*  Stack & heap sizes — overridable from the linker command line        */
/* -------------------------------------------------------------------- */

  .equ    Stack_Size,  0x00004000   /* 16 KB main stack */
  .equ    Heap_Size,   0x00002000   /*  8 KB FreeRTOS heap (heap_4) */

/* -------------------------------------------------------------------- */
/*  SRAM layout                                                          */
/* -------------------------------------------------------------------- */

  .section .bss
  .align  3
  .global __StackTop
  .global __StackLimit
  .global __HeapBase
  .global __HeapLimit

__StackLimit:
  .skip   Stack_Size
__StackTop:
  .skip   8                       /* 8-byte alignment for ART accelerator */

  .equ    __StackSize, Stack_Size

__HeapBase:
  .skip   Heap_Size
__HeapLimit:

/* -------------------------------------------------------------------- */
/*  Vector table                                                         */
/* -------------------------------------------------------------------- */

  .section .isr_vector, "a"
  .align  2
  .global __isr_vector

__isr_vector:
  .word   __StackTop                  /* 0x00  Initial SP          */
  .word   Reset_Handler               /* 0x04  Reset               */
  .word   NMI_Handler                 /* 0x08  NMI                 */
  .word   HardFault_Handler           /* 0x0C  Hard fault          */
  .word   MemManage_Handler           /* 0x10  MemManage           */
  .word   BusFault_Handler            /* 0x14  BusFault            */
  .word   UsageFault_Handler          /* 0x18  UsageFault          */
  .word   0                           /* 0x1C  Reserved            */
  .word   0                           /* 0x20  Reserved            */
  .word   0                           /* 0x24  Reserved            */
  .word   0                           /* 0x28  Reserved            */
  .word   vPortSVCHandler             /* 0x2C  SVCall -> FreeRTOS  */
  .word   DebugMon_Handler            /* 0x30  DebugMonitor        */
  .word   0                           /* 0x34  Reserved            */
  .word   xPortPendSVHandler          /* 0x38  PendSV -> FreeRTOS  */
  .word   xPortSysTickHandler         /* 0x3C  SysTick -> FreeRTOS */

/* External interrupts (subset we use) */
  .word   0                           /* WWDG */
  .word   0                           /* PVD */
  .word   0                           /* RTC */
  .word   0                           /* RTC_S */
  .word   0                           /* TAMP */
  .word   0                           /* RAMCFG */
  .word   0                           /* FLASH */
  .word   0                           /* GTZC */
  .word   0                           /* RCC */
  .word   0                           /* EXTI0 */
  .word   0                           /* EXTI1 */
  .word   0                           /* EXTI2 */
  .word   0                           /* EXTI3 */
  .word   0                           /* EXTI4 */
  .word   0                           /* DMA1_Channel0 */
  .word   0                           /* DMA1_Channel1 */
  .word   0                           /* DMA1_Channel2 */
  .word   0                           /* DMA1_Channel3 */
  .word   0                           /* DMA1_Channel4 */
  .word   0                           /* DMA1_Channel5 */
  .word   0                           /* DMA1_Channel6 */
  .word   0                           /* ADC1 */
  .word   0                           /* ADC2 */
  .word   0                           /* DAC1 */
  .word   0                           /* FDCAN1_IT0 */
  .word   0                           /* FDCAN1_IT1 */
  .word   0                           /* TIM1_BRK */
  .word   0                           /* TIM1_UP */
  .word   0                           /* TIM1_TRG_COM */
  .word   0                           /* TIM1_CC */
  .word   TIM2_IRQHandler             /* TIM2 */
  .word   0                           /* TIM3 */
  .word   0                           /* TIM4 */
  .word   0                           /* TIM5 */
  .word   0                           /* TIM6 */
  .word   0                           /* TIM7 */
  .word   0                           /* TIM8_BRK */
  .word   USART1_IRQHandler           /* USART1 */
  .word   USART2_IRQHandler           /* USART2 */
  .word   0                           /* USART3 */
  .word   0                           /* UART4 */
  .word   0                           /* UART5 */
  .word   0                           /* LPUART1 */
  .word   0                           /* LPTIM1 */
  .word   0                           /* LPTIM2 */
  .word   0                           /* I2C1 */
  .word   0                           /* I2C2 */
  .word   0                           /* I2C3 */
  .word   0                           /* I2C4 */
  .word   0                           /* SPI1 */
  .word   0                           /* SPI2 */
  .word   0                           /* SPI3 */
  .word   0                           /* IWDG1 */
  .word   0                           /* etc */

/* -------------------------------------------------------------------- */
/*  Reset_Handler                                                       */
/* -------------------------------------------------------------------- */

  .section .text.Reset_Handler
  .weak   Reset_Handler
  .type   Reset_Handler, %function

Reset_Handler:
  /* Copy .data from flash to SRAM */
  ldr   r0, =__etext
  ldr   r1, =__data_start__
  ldr   r2, =__data_end__
  cmp   r1, r2
  beq   .L_zero_bss
  movs  r3, #0
  b     .L_copy_loop
.L_copy_loop:
  ldrb  r4, [r0, r3]
  strb  r4, [r1, r3]
  adds  r3, r3, #1
  cmp   r2, r1
  add   r1, r1, #1
  bhi   .L_copy_loop

.L_zero_bss:
  ldr   r2, =__bss_start__
  ldr   r4, =__bss_end__
  movs  r3, #0
  b     .L_zero_loop
.L_zero_loop:
  str   r3, [r2]
  adds  r2, r2, #4
  cmp   r2, r4
  bcc   .L_zero_loop

  /* Run C++ constructors if any (none in this project, but safe) */
  bl    SystemInit
  bl    main
  b     .

  .size Reset_Handler, .-Reset_Handler

/* -------------------------------------------------------------------- */
/*  Default weak aliases                                                */
/* -------------------------------------------------------------------- */

  .weak   NMI_Handler
  .thumb_set NMI_Handler,Default_Handler

  .weak   HardFault_Handler
  .thumb_set HardFault_Handler,Default_Handler

  .weak   MemManage_Handler
  .thumb_set MemManage_Handler,Default_Handler

  .weak   BusFault_Handler
  .thumb_set BusFault_Handler,Default_Handler

  .weak   UsageFault_Handler
  .thumb_set UsageFault_Handler,Default_Handler

  .weak   DebugMon_Handler
  .thumb_set DebugMon_Handler,Default_Handler

  .weak   TIM2_IRQHandler
  .thumb_set TIM2_IRQHandler,Default_Handler

  .weak   USART1_IRQHandler
  .thumb_set USART1_IRQHandler,Default_Handler

  .weak   USART2_IRQHandler
  .thumb_set USART2_IRQHandler,Default_Handler

  .weak   xPortPendSVHandler
  .thumb_set xPortPendSVHandler,Default_Handler

  .weak   xPortSysTickHandler
  .thumb_set xPortSysTickHandler,Default_Handler

  .weak   vPortSVCHandler
  .thumb_set vPortSVCHandler,Default_Handler

  .section .text.Default_Handler,"ax",%progbits
Default_Handler:
  b   .
  .size Default_Handler, .-Default_Handler

/* -------------------------------------------------------------------- */
/*  C-runtime hook stubs                                                */
/* -------------------------------------------------------------------- */

  .weak   SystemInit
  .type   SystemInit, %function
SystemInit:
  /* Enable FPU if configured — for now leave disabled. */
  bx    lr
  .size SystemInit, .-SystemInit

  .end
