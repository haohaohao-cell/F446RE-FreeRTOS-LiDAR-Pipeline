#include "app.h"
#include "main.h"
#include "board_pins.h"
#include "lidar.h"
#include "imu.h"
#include "telemetry.h"
#include "app_rtos.h"
#include "motor.h"
#include "encoder.h"
#include "tim_cal.h"
#include "log.h"
#include "i2c_bus.h"
#include "oled.h"
#include "bt_rx.h"
#include "pose.h"

#include "cmsis_os2.h"
#include "FreeRTOS.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim6;

/* ========================================================================== */
/* boot-time helpers                                                          */
/* ========================================================================== */

static void dwt_init(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/* ========================================================================== */
/* debug console                                                              */
/* ========================================================================== */
/* Buffer size: CODING_STANDARDS 5.4 asks for <= 80 characters per status line.
 * 128 leaves room for the longer boot lines while keeping the stack cost of a
 * print bounded (~128 + vsnprintf frame), which matters because the 1 Hz block
 * is printed from a 1024-byte task stack.  A longer line is truncated, not
 * overrun - vsnprintf guarantees that, and the truncation is counted. */
#define DBG_LINE_MAX 128U

static uint32_t dbg_truncated;

void dbg_printf(const char *fmt, ...)
{
  char buf[DBG_LINE_MAX];
  va_list ap;
  int n;

  va_start(ap, fmt);
  n = vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);

  if (n <= 0)
  {
    return;
  }

  if (n >= (int)sizeof(buf))
  {
    /* Truncated: everything that fits was still produced, and the event is
     * counted instead of being invisible. */
    n = (int)sizeof(buf) - 1;
    dbg_truncated++;
  }

  if (osKernelGetState() == osKernelRunning)
  {
    /* Scheduler running: never block a task on the UART (CODING_STANDARDS 5.4).
     * The bytes go into a ring and leave through DMA; a full ring drops them
     * and counts log_drop. */
    (void)log_write(buf, (uint16_t)n);
  }
  else
  {
    /* Boot: single-threaded, and a blocking transmit is the only way to keep
     * the banner in order before any task exists. */
    (void)HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)n, 100U);
  }
}

/* One-time per-task banner, deliberately WITHOUT vsnprintf: the IMU task has a
 * 512-byte stack and a printf frame does not belong on it.  48 bytes of buffer
 * and a hand-rolled unsigned conversion keep this safe from any task. */
static char *u32_to_str(char *p, uint32_t v)
{
  char tmp[10];
  uint8_t n = 0U;

  if (v == 0U)
  {
    *p++ = '0';
    return p;
  }
  while ((v > 0U) && (n < sizeof(tmp)))
  {
    tmp[n] = (char)('0' + (char)(v % 10U));
    v /= 10U;
    n++;
  }
  while (n > 0U)
  {
    n--;
    *p++ = tmp[n];
  }
  return p;
}

void dbg_task_started(const char *name, uint32_t period_ms, uint32_t prio, uint32_t stack)
{
  char buf[64];
  char *p = buf;

  memcpy(p, "[task] ", 7U);
  p += 7;

  while ((*name != '\0') && (p < &buf[24]))
  {
    *p++ = *name++;
  }

  memcpy(p, " per=", 5U);
  p += 5;
  p = u32_to_str(p, period_ms);
  if (period_ms == 0U)
  {
    memcpy(p, "(event)", 7U);
    p += 7;
  }
  else
  {
    memcpy(p, "ms", 2U);
    p += 2;
  }

  memcpy(p, " prio=", 6U);
  p += 6;
  p = u32_to_str(p, prio);

  memcpy(p, " stk=", 5U);
  p += 5;
  p = u32_to_str(p, stack);

  memcpy(p, "\r\n", 2U);
  p += 2;

  (void)log_write(buf, (uint16_t)(p - buf));
}

/* ========================================================================== */
/* NVIC priorities                                                            */
/* ========================================================================== */
/* FreeRTOS rule: any ISR that calls a FreeRTOS/CMSIS-RTOS2 API must have a
 * priority number >= configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY (5 here).
 * On Cortex-M a *larger* number is a *lower* logical priority.
 *
 *    0  TIM7 (HAL time base)  - must stay 0, see HANDOVER 5.1
 *    5  USART3, DMA1_Stream1  - LiDAR: post to Q_Lidar
 *    5  TIM6                  - IMU cadence: osSemaphoreRelease   (was 7)
 *    5  UART5                 - BT receive: bt_rx + semaphore     (was 6)
 *    5  DMA1_Stream0          - BT receive DMA
 *    6  I2C1 EV/ER           - no RTOS call (transfers are synchronous now)
 *    6  DMA1_Stream6/7       - debug log TX / telemetry TX, no RTOS call
 *    7  USART2               - polled only
 *
 * This function runs LAST in app_init() because HAL_UART_MspInit(),
 * HAL_I2C_MspInit(), HAL_TIM_Base_MspInit(), telemetry_dma_init() and
 * i2c_bus_recover() all write priority registers themselves and the last writer
 * wins (HANDOVER 5.3). */
static void irq_priorities(void)
{
  HAL_NVIC_SetPriority(USART3_IRQn, BOARD_PRIO_LIDAR, 0U);
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, BOARD_PRIO_LIDAR, 0U);

  HAL_NVIC_SetPriority(TIM6_DAC_IRQn, BOARD_PRIO_FAST_TICK, 0U);
  HAL_NVIC_SetPriority(UART5_IRQn, BOARD_PRIO_BT_RX, 0U);
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, BOARD_PRIO_BT_RX, 0U);

  HAL_NVIC_SetPriority(I2C1_EV_IRQn, BOARD_PRIO_I2C, 0U);
  HAL_NVIC_SetPriority(I2C1_ER_IRQn, BOARD_PRIO_I2C, 0U);
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, BOARD_PRIO_BT_TX_DMA, 0U);
  HAL_NVIC_SetPriority(DMA1_Stream7_IRQn, BOARD_PRIO_BT_TX_DMA, 0U);

  HAL_NVIC_SetPriority(USART2_IRQn, BOARD_PRIO_CONSOLE, 0U);
}

/* ========================================================================== */
/* application init                                                           */
/* ========================================================================== */
void app_init(void)
{
  int imu_status;
  int oled_status;
  int bt_status;
  int i2c_status;
  int log_status;
  tim_cal_result_t cal_motor;
  tim_cal_result_t cal_fast;

  dwt_init();          /* must be first: everything below times itself with DWT */

  /* Motor and encoder hardware first: motor_init() starts with both H-bridge
   * inputs low and 0 % duty, so the car cannot move while the rest boots. */
  motor_init();
  encoder_init();

  /* RTOS objects before any driver needs one: the I2C mutex is created inside
   * i2c_bus_init(), which requires an initialised kernel, and Q_Lidar must
   * exist before lidar_init() arms the DMA reception. */
  app_rtos_init();

  i2c_status = i2c_bus_init();

  /* Console transport (USART2 TX DMA).  Must precede the first dbg_printf that
   * wants to be non-blocking; the boot banner still uses the blocking path. */
  log_status = log_init();

  /* Telemetry first: its autobaud probe reconfigures UART5, which would throw
   * away the DMA reception that bt_rx_init() installs.  Order matters. */
  telemetry_init();

  imu_status = imu_init();
  oled_status = oled_init();

  pose_init();

  bt_status = bt_rx_init();

  /* Now that every driver has configured its own interrupts, re-assert the one
   * authoritative table. */
  irq_priorities();

  /* Trim the HAL time base to exactly 1 kHz, measured against DWT. */
  hal_timebase_calibrate();

  /* Part 2 time bases, for the same reason: this board's APB1 timer clock does
   * not match the reference manual (HANDOVER 5.2).
   *   - PWM timers: nominal 20 kHz.  TIM2 inherits TIM4's calibrated prescaler.
   *   - TIM6: nominal 100 Hz, i.e. the 10 ms control cadence.
   * Both MUST be correct before osKernelStart() and, for TIM6, before
   * imu_start() arms the update interrupt. */
  (void)hal_tim_calibrate(motor_left_timer_handle(), BOARD_MOTOR_PWM_HZ, &cal_motor);
  motor_sync_pwm_clock();
  (void)hal_tim_calibrate(&htim6, 100U, &cal_fast);

  /* Starts continuous UART + DMA reception and sends the SCAN command. */
  lidar_init();

  dbg_printf("\r\nBlock4.5 Part2 ready imu=%d oled=%d i2c=%d bt=%d log=%d\r\n",
             imu_status, oled_status, i2c_status, bt_status, log_status);

  dbg_printf("timebase: psc=%lu arr=%lu calib=%lu/%lu timpre=%lu\r\n",
             (unsigned long)(TIM7->PSC),
             (unsigned long)(TIM7->ARR),
             (unsigned long)hal_timebase_calib_irqs,
             (unsigned long)hal_timebase_calib_expect,
             (unsigned long)hal_timebase_timpre_found);

  /* Part 2 time bases: nominal / measured / corrected / effective, so a
   * mismatch is visible in a captured boot log instead of being silent. */
  dbg_printf("pwm clk: nom=%lu meas=%lu psc %lu->%lu upd %lu->%luHz%s\r\n",
             (unsigned long)cal_motor.nominal_hz,
             (unsigned long)cal_motor.measured_clk_hz,
             (unsigned long)cal_motor.psc_before,
             (unsigned long)cal_motor.psc_after,
             (unsigned long)cal_motor.update_hz_before,
             (unsigned long)cal_motor.update_hz_after,
             (cal_motor.corrected != 0U) ? " corr" : "");

  dbg_printf("fast tick: nom=%lu meas=%lu psc %lu->%lu upd %lu->%luHz%s\r\n",
             (unsigned long)cal_fast.nominal_hz,
             (unsigned long)cal_fast.measured_clk_hz,
             (unsigned long)cal_fast.psc_before,
             (unsigned long)cal_fast.psc_after,
             (unsigned long)cal_fast.update_hz_before,
             (unsigned long)cal_fast.update_hz_after,
             (cal_fast.corrected != 0U) ? " corr" : "");

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

  /* The two facts a reader of this log must not have to guess:
   *   - which fields below are still unmeasured (all 0 / TBD on hardware), and
   *   - that the watchdog is not enabled yet.
   * Kept ASCII on purpose: a captured log goes through cmd.exe and the PC tools,
   * where non-ASCII text turns into mojibake on a non-UTF-8 code page. */
  dbg_printf("note: WD=0 (IWDG off, TBD on HW); wheel dia/track TBD on HW\r\n");

  /* TIM6 last: it is the 10 ms cadence, and every tick released before the
   * scheduler starts is one credit the IMU task can never consume (they would
   * show up as imu_ticks_unused in the very first status line).  Starting it at
   * the very end of init leaves at most one tick pending. */
  imu_start();
}

/* ========================================================================== */
/* interrupt callbacks (HAL weak overrides)                                   */
/* ========================================================================== */

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    lidar_on_half();      /* -> Q_Lidar */
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    lidar_on_full();      /* -> Q_Lidar */
  }
}

/* UART5 receive-to-idle: the DMA window holds `Size` bytes.
 * ISR work is: copy into the ring (bt_rx), then wake the BT RX task.  Parsing
 * and queueing happen in the task. */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  if (huart->Instance == UART5)
  {
    bt_rx_isr_event(Size);
    app_rtos_bt_rx_event();
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART2)
  {
    log_on_tx_complete();       /* chain the next queued log chunk */
  }
  else if (huart->Instance == UART5)
  {
    telemetry_on_tx_complete();
  }
  else
  {
    /* USART3 TX is not used for anything asynchronous. */
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    lidar_on_uart_error();  /* -> Q_Lidar, parser task re-arms */
  }
  else if (huart->Instance == UART5)
  {
    /* Two independent users of one UART: TX (telemetry) and RX (commands).
     * Both get told; each repairs only its own direction, in task context
     * (CODING_STANDARDS 5.2). */
    telemetry_on_error();
    bt_rx_on_error();
  }
  else
  {
    /* USART2 (console): nothing to repair, it is polled. */
  }
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
  if (hi2c->Instance == I2C1)
  {
    /* Interrupt context: count only.  Transfers are synchronous now, so the
     * failure is already reported by the i2c_bus_* return value and the bus
     * recovery runs in task context. */
    imu_on_error();
  }
}
