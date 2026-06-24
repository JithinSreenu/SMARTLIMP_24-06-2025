# FreeRTOS Kernel — copy these files from the official FreeRTOS-Kernel release

This directory mirrors the project layout from the brief. The exact files
needed are listed below. They are pulled from the official
**FreeRTOS-Kernel V10.x.x** release (MIT licence) on the online PC and
copied over via USB.

## Tree to copy

```
Third_Party/FreeRTOS/Source/
├── include/                              ← copy every *.h from the release
│   ├── FreeRTOS.h
│   ├── task.h
│   ├── queue.h
│   ├── list.h
│   ├── timers.h
│   ├── semphr.h
│   ├── event_groups.h
│   ├── croutine.h
│   ├── stream_buffer.h
│   ├── message_buffer.h
│   ├── portable.h
│   ├── projdefs.h
│   ├── deprecated_definitions.h
│   └── mpu_wrappers.h
├── tasks.c                               ← kernel task code
├── queue.c                               ← queue / semaphore / mutex
├── list.c                                ← scheduler list
├── timers.c                              ← software timers
├── event_groups.c                        ← event groups
├── stream_buffer.c                       ← stream buffers
├── portable/
│   ├── GCC/
│   │   └── ARM_CM33_NTZ/
│   │       ├── port.c                    ← M33 non-trustzone port
│   │       └── portasm.s                 ← context switch in asm
│   └── MemMang/
│       └── heap_4.c                      ← coalescing allocator (only this one)
```

## Patches to apply after copying

1. In `port.c` open the function `vPortSetupTimerInterrupt()` and verify:
   ```c
   configCPU_CLOCK_HZ              → 16000000  (16 MHz MSI)
   configTICK_RATE_HZ              → 1000      (matches FreeRTOSConfig.h)
   ```
2. In `portasm.s` make sure `pxCurrentTCB` is declared `volatile`.

That's it. Do NOT add heap_1, heap_2, heap_3, heap_5 — only `heap_4.c`.
Do NOT add the ARM_CM4F, ARM_CM7, ARM_CM23 ports — only ARM_CM33_NTZ.
