#include "app.h"
#include "main.h"
#include "lidar.h"
#include "imu.h"
#include "telemetry.h"
#include "app_rtos.h"

#include "FreeRTOS.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern I2C_HandleTypeDef hi2c1;

/* ========================================================================== */
/* boot-time helpers                                                          */
/* ========================================================================== */

static void dwt_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void dbg_printf(const char *fmt, ...)
{
  char buf[256];
  va_list ap;
  int n;

  va_start(ap, fmt);
  n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  if (n > 0)
  {
    if (n >= (int)sizeof(buf)) { n = (int)sizeof(buf) - 1; }
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)n, 100U);
  }
}

static void motors_safe(void)
{
  GPIO_InitTypeDef g = {0};

  __HAL_RCC_GPIOB_CLK_ENABLE();
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6 | GPIO_PIN_7, GPIO_PIN_RESET);
  g.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  g.Mode = GPIO_MODE_OUTPUT_PP;
  g.Pull = GPIO_NOPULL;
  g.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &g);
}

/* -----------------------------------------------------------------------------
 * NVIC priorities.
 *
 * FreeRTOS rule: any ISR that calls a FreeRTOS API must have a priority number
 * >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5 here).  Remember that on
 * Cortex-M a *larger* number means a *lower* logical priority.
 *
 *   5  USART3, DMA1_Stream1  <- LiDAR: these ISRs post to Queue 1
 *   6  I2C1, UART5, DMA1_Stream7
 *   7  USART2, TIM6 (IMU sampling)
 *  15  TIM7 (HAL time base, set in stm32f4xx_hal_timebase_tim.c)
 *  15  SysTick / PendSV (set by the FreeRTOS port)
 *
 * Setting the LiDAR interrupts to a number below 5 would trip
 * vPortValidateInterruptPriority() the first time the ISR calls
 * osMessageQueuePut() - the classic FreeRTOS + STM32 mistake.
 * -------------------------------------------------------------------------- */
#define IRQ_PRIO_LIDAR     5U
#define IRQ_PRIO_MID       6U
#define IRQ_PRIO_LOW       7U

static void irq_priorities(void)
{
  HAL_NVIC_SetPriority(USART3_IRQn, IRQ_PRIO_LIDAR, 0U);
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, IRQ_PRIO_LIDAR, 0U);

  HAL_NVIC_SetPriority(I2C1_EV_IRQn, IRQ_PRIO_MID, 0U);
  HAL_NVIC_SetPriority(I2C1_ER_IRQn, IRQ_PRIO_MID, 0U);
  HAL_NVIC_SetPriority(UART5_IRQn, IRQ_PRIO_MID, 0U);
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, IRQ_PRIO_MID, 0U);

  HAL_NVIC_SetPriority(USART2_IRQn, IRQ_PRIO_LOW, 0U);
  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, IRQ_PRIO_LOW, 0U);
}

/* ========================================================================== */
/* application init                                                           */
/* ========================================================================== */

void app_init(void)
{
  int imu_status;

  dwt_init();          /* must be first: everything below times itself with DWT */
  motors_safe();

  telemetry_init();
  imu_status = imu_init();
  imu_start();

  /* Re-assert the final NVIC map *after* every driver has had its say:
   * HAL_xxx_MspInit(), telemetry_dma_init() and i2c_bus_recover() all write
   * NVIC priority registers themselves, and the last writer wins. */
  irq_priorities();

  /* Trim the HAL time base to exactly 1 kHz, measured against DWT (see
   * stm32f4xx_hal_timebase_tim.c).  Must run before anything relies on
   * HAL_GetTick(). */
  hal_timebase_calibrate();

  /* Create both queues and both tasks.  This MUST happen before lidar_init(),
   * because lidar_init() starts the circular DMA reception and the very first
   * half-complete interrupt will already try to post into Queue 1. */
  app_rtos_init();

  /* Starts continuous UART + DMA reception and sends the SCAN command. */
  lidar_init();

  dbg_printf("\r\nBlock4.5 FreeRTOS LiDAR pipeline ready (imu=%d hal=%lu "
             "err=0x%lx who=0x%02lx, bt_baud=%lu)\r\n",
             imu_status,
             (unsigned long)imu_diag_status,
             (unsigned long)imu_diag_err,
             (unsigned long)imu_diag_who,
             (unsigned long)telemetry_bt_baud());

  /* Time-base report.  TIM7 is now trimmed so that
   *     TIM7CLK / (PSC+1) / (ARR+1) == 1000
   * The "implied" clock is what the prescaler mathematically corresponds to;
   * calib_irqs/calib_expect is the raw measurement the prescaler was derived
   * from (~20 ticks expected over 20 ms). */
  dbg_printf("timebase: sysclk=%lu hclk=%lu pclk1=%lu tim7 psc=%lu arr=%lu "
             "implied_clk=%lu calib=%lu/%lu timpre=%lu\r\n",
             (unsigned long)SystemCoreClock,
             (unsigned long)HAL_RCC_GetHCLKFreq(),
             (unsigned long)HAL_RCC_GetPCLK1Freq(),
             (unsigned long)(TIM7->PSC),
             (unsigned long)(TIM7->ARR),
             (unsigned long)(1000UL * (TIM7->PSC + 1UL) * (TIM7->ARR + 1UL)),
             (unsigned long)hal_timebase_calib_irqs,
             (unsigned long)hal_timebase_calib_expect,
             (unsigned long)hal_timebase_timpre_found);

  /* Verified: 50 ms of DWT cycles must advance HAL_GetTick() by 50. */
  {
    uint32_t c0 = DWT->CYCCNT;
    uint32_t h0 = HAL_GetTick();
    uint32_t span_cycles = SystemCoreClock / 20UL;
    uint32_t dh;

    while ((uint32_t)(DWT->CYCCNT - c0) < span_cycles)
    {
    }
    dh = HAL_GetTick() - h0;

    dbg_printf("timebase check: 50 ms (dwt) -> %lu ms (hal) %s\r\n",
               (unsigned long)dh,
               ((dh >= 48U) && (dh <= 52U)) ? "OK" : "MISMATCH");
  }
}

/* ========================================================================== */
/* interrupt callbacks (HAL weak overrides)                                   */
/* ========================================================================== */

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    lidar_on_half();      /* -> Queue 1 */
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    lidar_on_full();      /* -> Queue 1 */
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    lidar_on_uart_error();  /* -> Queue 1, parser task re-arms */
  }
  else if (huart->Instance == UART5)
  {
    telemetry_on_error();
  }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    imu_on_error();
  }
}
