/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    FreeRTOSConfig.h
  * @brief   FreeRTOS 10.3.1 kernel configuration for
  *          Block 4.5 Coding Day - FreeRTOS LiDAR Data Pipeline
  *
  *          Target : STM32F446RE (Cortex-M4F), 168 MHz, 128 KB SRAM
  *          RTOS   : FreeRTOS 10.3.1 + CMSIS-RTOS API v2 (cmsis_os2.c)
  *
  *          Interrupt priority design (see report section 5):
  *            - configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5
  *              => only ISRs whose NVIC priority number is >= 5 (i.e. logically
  *                 at or below that level) may call any FreeRTOS "FromISR" API.
  *            - USART3 / DMA1_Stream1 (LiDAR)  -> priority 5   (allowed to post)
  *            - I2C1 / UART5 / USART2          -> priority 6..7 (no RTOS calls)
  *            - HAL time base (TIM7)           -> priority 15 (never RTOS)
  *            - PendSV / SysTick (kernel)      -> priority 15 (lowest)
  ******************************************************************************
  */
/* USER CODE END Header */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/* The CMSIS-RTOS v2 layer (freertos_os2.h) does "#include CMSIS_device_header"
 * and needs IRQn_Type / SysTick / NVIC_* from the device header.  Also gives us
 * the "extern uint32_t SystemCoreClock" declaration used by configCPU_CLOCK_HZ
 * when port.c is compiled on its own. */
#ifndef CMSIS_device_header
#define CMSIS_device_header "stm32f4xx.h"
#endif
#include CMSIS_device_header

/*-----------------------------------------------------------
 * Application specific definitions.
 *----------------------------------------------------------*/

#define configUSE_PREEMPTION                     1
/* Must be 0: the CMSIS-RTOS v2 osPriority_t enum spans 56 levels, and the
 * optimised (CLZ based) task selection only supports <= 32 priorities. */
#define configUSE_PORT_OPTIMISED_TASK_SELECTION  0
#define configUSE_TICKLESS_IDLE                  0
#define configCPU_CLOCK_HZ                       (SystemCoreClock)
#define configTICK_RATE_HZ                       ((TickType_t)1000)
#define configMAX_PRIORITIES                     (56)
#define configMINIMAL_STACK_SIZE                 ((uint16_t)128)
#define configMAX_TASK_NAME_LEN                  (16)
#define configUSE_16_BIT_TICKS                   0
#define configIDLE_SHOULD_YIELD                  1
#define configUSE_TASK_NOTIFICATIONS             1
#define configUSE_MUTEXES                        1
#define configUSE_RECURSIVE_MUTEXES              1
#define configUSE_COUNTING_SEMAPHORES            1
#define configQUEUE_REGISTRY_SIZE                8
#define configUSE_QUEUE_SETS                     0
#define configUSE_TIME_SLICING                   1
#define configUSE_NEWLIB_REENTRANT               0
#define configENABLE_BACKWARD_COMPATIBILITY      0
#define configNUM_THREAD_LOCAL_STORAGE_POINTERS  0
#define configSTACK_DEPTH_TYPE                   uint16_t
#define configMESSAGE_BUFFER_LENGTH_TYPE         size_t

/* Memory allocation: heap_4 (first fit + adjacent block coalescing) */
#define configSUPPORT_STATIC_ALLOCATION          0
#define configSUPPORT_DYNAMIC_ALLOCATION         1
#define configTOTAL_HEAP_SIZE                    ((size_t)(32 * 1024))
#define configAPPLICATION_ALLOCATED_HEAP         0

/* Hook / debug support */
#define configUSE_IDLE_HOOK                      0
#define configUSE_TICK_HOOK                      0
#define configCHECK_FOR_STACK_OVERFLOW           2
#define configUSE_MALLOC_FAILED_HOOK             1
#define configUSE_DAEMON_TASK_STARTUP_HOOK       0

/* Run-time stats / trace */
#define configGENERATE_RUN_TIME_STATS            0
#define configUSE_TRACE_FACILITY                 1
#define configUSE_STATS_FORMATTING_FUNCTIONS     0

/* Co-routines are not used */
#define configUSE_CO_ROUTINES                    0
#define configMAX_CO_ROUTINE_PRIORITIES          (2)

/* Software timers: required by the CMSIS-RTOS v2 layer (freertos_os2.h) */
#define configUSE_TIMERS                         1
#define configTIMER_TASK_PRIORITY                (configMAX_PRIORITIES - 1)
#define configTIMER_QUEUE_LENGTH                 10
#define configTIMER_TASK_STACK_DEPTH             (configMINIMAL_STACK_SIZE * 2)

/*-----------------------------------------------------------
 * Interrupt nesting behaviour (Cortex-M4, 4 priority bits)
 *----------------------------------------------------------*/
#ifdef __NVIC_PRIO_BITS
  #define configPRIO_BITS                        __NVIC_PRIO_BITS
#else
  #define configPRIO_BITS                        4
#endif

/* The lowest interrupt priority that can be used in a call to a "set priority"
   function. */
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY         15

/* The highest interrupt priority that can be used by any interrupt service
   routine that makes calls to interrupt safe FreeRTOS API functions.
   NOTE: numerically LARGER values are logically LOWER priority. */
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY    5

/* Interrupt priorities used by the kernel port layer itself. */
#define configKERNEL_INTERRUPT_PRIORITY \
  (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* !!! configMAX_SYSCALL_INTERRUPT_PRIORITY must not be set to zero !!! */
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
  (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/*-----------------------------------------------------------
 * Optional API functions
 *----------------------------------------------------------*/
#define INCLUDE_vTaskPrioritySet                 1
#define INCLUDE_uxTaskPriorityGet                1
#define INCLUDE_vTaskDelete                      1
#define INCLUDE_vTaskSuspend                     1
#define INCLUDE_vTaskDelayUntil                  1
#define INCLUDE_vTaskDelay                       1
#define INCLUDE_xTaskGetSchedulerState           1
#define INCLUDE_xTaskGetCurrentTaskHandle        1
#define INCLUDE_uxTaskGetStackHighWaterMark      1
#define INCLUDE_xTaskGetIdleTaskHandle           1
#define INCLUDE_eTaskGetState                    1
#define INCLUDE_xTimerPendFunctionCall           1
#define INCLUDE_xTaskAbortDelay                  1
#define INCLUDE_xTaskGetHandle                   1
#define INCLUDE_xQueueGetMutexHolder             1
#define INCLUDE_xSemaphoreGetMutexHolder         1

/*-----------------------------------------------------------
 * CMSIS-RTOS v2 glue
 *----------------------------------------------------------*/

/* Map the FreeRTOS port exception handlers onto the CMSIS standard names.
   NOTE: SysTick_Handler is deliberately NOT mapped here - it is provided by
   cmsis_os2.c (USE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION defaults to 0), which
   also clears the SysTick overflow flag and only forwards to the kernel once
   the scheduler has been started. */
#define vPortSVCHandler                          SVC_Handler
#define xPortPendSVHandler                       PendSV_Handler

/* The CMSIS-RTOS v2 layer implements the timer API, so timers must be enabled
   (see freertos_os2.h). */
#define USE_CUSTOM_SYSTICK_HANDLER_IMPLEMENTATION 0

/* Heap model selected in portable/MemMang (used by cmsis_os2.c) */
#define USE_FreeRTOS_HEAP_4

/*-----------------------------------------------------------
 * Assertion
 *----------------------------------------------------------*/
#define configASSERT(x) \
  if ((x) == 0) { taskDISABLE_INTERRUPTS(); for (;;) { } }

#endif /* FREERTOS_CONFIG_H */
