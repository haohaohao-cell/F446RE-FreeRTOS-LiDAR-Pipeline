#ifndef APP_H
#define APP_H

#include <stdint.h>

/* Brings the peripherals up and creates the FreeRTOS objects.
 * Called from main() before the scheduler is started. */
void app_init(void);

/* Blocking, polling UART transmit on USART2 (debug console).
 * Only ever called from task context.
 *
 * Behaviour depends on the scheduler state: before osKernelStart() it transmits
 * synchronously (the boot banner must not be reordered); afterwards the text is
 * queued into the log ring and leaves through TX DMA, so it never blocks the
 * caller (CODING_STANDARDS 5.4).  Lines longer than DBG_LINE_MAX are truncated
 * and counted. */
void dbg_printf(const char *fmt, ...);

/* One-time "this task is alive" banner: name, period (0 = event driven),
 * priority and stack.  Deliberately does not use vsnprintf so it is safe on the
 * 512-byte IMU task stack. */
void dbg_task_started(const char *name, uint32_t period_ms, uint32_t prio, uint32_t stack);

/* 1 if RCC_DCKCFGR.TIMPRE was asserted and got cleared by the HAL time-base
 * setup (see stm32f4xx_hal_timebase_tim.c).  Purely diagnostic. */
extern uint32_t hal_timebase_timpre_found;
extern volatile uint32_t hal_timebase_irq_count;
extern uint32_t hal_timebase_calib_irqs;
extern uint32_t hal_timebase_calib_expect;

/* Corrects the TIM7 prescaler against DWT so the HAL tick is exactly 1 kHz.
 * Call once, after the clocks are final and DWT is enabled. */
void hal_timebase_calibrate(void);

#endif
